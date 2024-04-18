#include "Network.h"
#include "Falcor.h"
namespace Mogwai
{
constexpr int kRingDepth = 4096;

struct WebSocketMessage
{
    char* payload = nullptr;
    size_t len = 0;
    char binary = 0;
    char first = 0;
    char final = 0;

    WebSocketMessage() = default;
    WebSocketMessage(char* _payload, size_t _len, char _binary, char _first, char _final)
        : payload(_payload), len(_len), binary(_binary), first(_first), final(_final)
    {}
    /** Allocate and copy the payload.
     */
    void setPayload(fstd::span<char> _payload)
    {
        if (payload)
        {
            Falcor::logWarning("WebSocketMessage::payload not null, freeing it");
            delete payload;
            payload = nullptr;
        }
        payload = new char[_payload.size()];
        len = _payload.size();
        memcpy(payload, _payload.data(), len);
    }

    static void destroy(void* _msg)
    {
        WebSocketMessage* msg = (WebSocketMessage*)_msg;
        free(msg->payload);
        msg->payload = nullptr;
        msg->len = 0;
    }

    /** Write payload message to the socket.
     *  Automatically allocate LWS_PRE header space.
     */
    int write(struct lws* wsi)
    {
        if (!payload)
        {
            Falcor::logError("WebSocketMessage::payload is null");
            return -1;
        }

        int flags = lws_write_ws_flags(binary ? LWS_WRITE_BINARY : LWS_WRITE_TEXT, first, final);
        std::vector<char> buffer(len + LWS_PRE);
        memcpy(buffer.data() + LWS_PRE, payload, len);
        return lws_write(wsi, (unsigned char*)buffer.data() + LWS_PRE, len, (lws_write_protocol)flags);
    }
};
struct SessionData
{
    struct lws_ring* ring;
    uint32_t msglen;
    uint32_t tail;
    uint8_t completed : 1;
    uint8_t flow_controlled : 1;
    uint8_t write_consume_pending : 1;
};

struct VHostStorage
{
    struct lws_context* context;
    struct lws_vhost* vhost;
};

int callback_minimal_server_echo(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len)
{
    SessionData* pss = (SessionData*)user;
    VHostStorage* vhd = (VHostStorage*)lws_protocol_vh_priv_get(lws_get_vhost(wsi), lws_get_protocol(wsi));
    // Request message data
    WebSocketMessage amsg;
    int n;

    switch (reason)
    {
    case LWS_CALLBACK_PROTOCOL_INIT:
        vhd = (VHostStorage*)lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi), lws_get_protocol(wsi), sizeof(VHostStorage));
        if (!vhd)
            return -1;

        vhd->context = lws_get_context(wsi);
        vhd->vhost = lws_get_vhost(wsi);

        break;

    case LWS_CALLBACK_ESTABLISHED:
        /* generate a block of output before travis times us out */
        Falcor::logInfo("LWS_CALLBACK_ESTABLISHED");
        pss->ring = lws_ring_create(sizeof(WebSocketMessage), kRingDepth, WebSocketMessage::destroy);
        if (!pss->ring)
            return 1;
        pss->tail = 0;
        break;

    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
        if (pss->write_consume_pending)
        {
            /* perform the deferred fifo consume */
            lws_ring_consume_single_tail(pss->ring, &pss->tail, 1);
            pss->write_consume_pending = 0;
        }

        const auto* pmsg = (WebSocketMessage*)lws_ring_get_element(pss->ring, &pss->tail);
        if (!pmsg)
        {
            break;
        }

        NetworkServer* server = (NetworkServer*)lws_vhost_user(lws_get_vhost(wsi));
        std::string response = server->dispatch((const char*)pmsg->payload);

        amsg = WebSocketMessage(const_cast<char*>(response.c_str()), response.size(), false, true, true);

        int m = amsg.write(wsi);
        if (m < (int)amsg.len)
        {
            Falcor::logError("Incomplete write: {}/{}", m, (int)amsg.len);
            return -1;
        }
        /*
         * Workaround deferred deflate in pmd extension by only
         * consuming the fifo entry when we are certain it has been
         * fully deflated at the next WRITABLE callback.  You only need
         * this if you're using pmd.
         */
        pss->write_consume_pending = 1;
        lws_callback_on_writable(wsi);

        if (pss->flow_controlled && (int)lws_ring_get_count_free_elements(pss->ring) > kRingDepth - 5)
        {
            lws_rx_flow_control(wsi, 1);
            pss->flow_controlled = 0;
        }

        if (pmsg && pmsg->final)
            pss->completed = 1;

        break;
    }

    case LWS_CALLBACK_RECEIVE:
        amsg.first = (char)lws_is_first_fragment(wsi);
        amsg.final = (char)lws_is_final_fragment(wsi);
        amsg.binary = (char)lws_frame_is_binary(wsi);
        n = (int)lws_ring_get_count_free_elements(pss->ring);
        if (!n)
        {
            Falcor::logWarning("No space in the ring, dropping");
            break;
        }

        if (amsg.final)
            pss->msglen = 0;
        else
            pss->msglen += (uint32_t)len;

        amsg.len = len;
        amsg.setPayload(fstd::span<char>((char*)in, len));
        if (!lws_ring_insert(pss->ring, &amsg, 1))
        {
            WebSocketMessage::destroy(&amsg);
            Falcor::logWarning("No space in the ring, dropping");
            break;
        }
        lws_callback_on_writable(wsi);

        if (n < 3 && !pss->flow_controlled)
        {
            pss->flow_controlled = 1;
            lws_rx_flow_control(wsi, 0);
        }
        break;

    case LWS_CALLBACK_CLOSED:
        Falcor::logInfo("LWS_CALLBACK_CLOSED");
        lws_ring_destroy(pss->ring);

        lws_cancel_service(lws_get_context(wsi));

        break;

    default:
        break;
    }

    return 0;
}

const lws_protocols kProtocols[] = {
    {"lws-minimal-server-echo", callback_minimal_server_echo, sizeof(struct SessionData), 1024, 0, nullptr, 0},
    LWS_PROTOCOL_LIST_TERM};

NetworkServer::NetworkServer(const std::string& host, int port, Clock& clock) : mClock(clock)
{
    // Init LWS server context
    lws_context_creation_info info{};
    info.port = port;
    info.server_string = "MogwaiWS";
    info.options = LWS_SERVER_OPTION_VALIDATE_UTF8;
    info.protocols = kProtocols;
    info.user = this;

    mpWebSocketContext = lws_create_context(&info);
    mpServerThread = std::make_unique<std::thread>(
        [this]()
        {
            int ret{1};
            while (ret >= 0 && !mServerStop)
            {
                ret = lws_service(mpWebSocketContext, 0);
            }
            lws_context_destroy(mpWebSocketContext);
        }
    );
    Falcor::logInfo("Serving at ws://{}:{}", host, port);
}

std::string NetworkServer::dispatch(const std::string& clientMsg) const
{
    return std::to_string(getSceneAnimationTime());
}

NetworkServer::~NetworkServer()
{
    mServerStop = true;
    mpServerThread->detach();
}
double NetworkServer::getSceneAnimationTime() const
{
    return mClock.getTime();
}

} // namespace Mogwai
