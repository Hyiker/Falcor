#include "Network.h"
#include "Falcor.h"
namespace Mogwai
{
constexpr int kRingDepth = 4096;

struct WebSocketMessage
{
    void* payload; /* is malloc'd */
    size_t len;
    char binary;
    char first;
    char final;
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

static void __minimal_destroy_message(void* _msg)
{
    struct WebSocketMessage* msg = (struct WebSocketMessage*)_msg;

    free(msg->payload);
    msg->payload = NULL;
    msg->len = 0;
}

int callback_minimal_server_echo(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len)
{
    SessionData* pss = (SessionData*)user;
    VHostStorage* vhd = (VHostStorage*)lws_protocol_vh_priv_get(lws_get_vhost(wsi), lws_get_protocol(wsi));
    const WebSocketMessage* pmsg;
    WebSocketMessage amsg;
    int m, n, flags;

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
        pss->ring = lws_ring_create(sizeof(struct WebSocketMessage), kRingDepth, __minimal_destroy_message);
        if (!pss->ring)
            return 1;
        pss->tail = 0;
        break;

    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
        Falcor::logInfo("LWS_CALLBACK_SERVER_WRITEABLE");

        if (pss->write_consume_pending)
        {
            /* perform the deferred fifo consume */
            lws_ring_consume_single_tail(pss->ring, &pss->tail, 1);
            pss->write_consume_pending = 0;
        }

        pmsg = (WebSocketMessage*)lws_ring_get_element(pss->ring, &pss->tail);
        if (!pmsg)
        {
            Falcor::logInfo(" (nothing in ring)");
            break;
        }

        flags = lws_write_ws_flags(pmsg->binary ? LWS_WRITE_BINARY : LWS_WRITE_TEXT, pmsg->first, pmsg->final);

        NetworkServer* server = (NetworkServer*)lws_vhost_user(lws_get_vhost(wsi));
        std::string response = server->dispatch((const char*)pmsg->payload);
        std::vector<char> responseBuffer(response.begin(), response.end());
        responseBuffer.insert(responseBuffer.begin(), LWS_PRE, char(0));

        /* notice we allowed for LWS_PRE in the payload already */
        m = lws_write(wsi, ((unsigned char*)responseBuffer.data()) + LWS_PRE, responseBuffer.size() - LWS_PRE, (lws_write_protocol)flags);
        if (m < (int)responseBuffer.size() - LWS_PRE)
        {
            Falcor::logError("{} writing to ws socket", m);
            return -1;
        }

        Falcor::logInfo("wrote {}: flags: 0x{:x} first: {} final {}", m, flags, pmsg->first, pmsg->final);
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

        Falcor::logInfo(
            "LWS_CALLBACK_RECEIVE: {0:4d} (rpp {1:5d}, first {2:d}, last {3:d}, bin {4:d}, msglen {5:d} (+ {6:d} = {7:d}))",
            static_cast<int>(len),
            static_cast<int>(lws_remaining_packet_payload(wsi)),
            lws_is_first_fragment(wsi),
            lws_is_final_fragment(wsi),
            lws_frame_is_binary(wsi),
            pss->msglen,
            static_cast<int>(len),
            static_cast<int>(pss->msglen) + static_cast<int>(len)
        );

        if (len)
        {
            ;
            // puts((const char *)in);
            // lwsl_hexdump_notice(in, len);
        }

        amsg.first = (char)lws_is_first_fragment(wsi);
        amsg.final = (char)lws_is_final_fragment(wsi);
        amsg.binary = (char)lws_frame_is_binary(wsi);
        n = (int)lws_ring_get_count_free_elements(pss->ring);
        if (!n)
        {
            Falcor::logInfo("dropping!");
            break;
        }

        if (amsg.final)
            pss->msglen = 0;
        else
            pss->msglen += (uint32_t)len;

        amsg.len = len;
        /* notice we over-allocate by LWS_PRE */
        amsg.payload = malloc(LWS_PRE + len);
        if (!amsg.payload)
        {
            Falcor::logInfo("OOM: dropping");
            break;
        }

        memcpy((char*)amsg.payload + LWS_PRE, in, len);
        if (!lws_ring_insert(pss->ring, &amsg, 1))
        {
            __minimal_destroy_message(&amsg);
            Falcor::logInfo("dropping!");
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

NetworkServer::NetworkServer(const std::string& host, int port)
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

std::string NetworkServer::dispatch(const char* requestPayload) const
{
    static int count = 0;

    return fmt::format("{}", count++);
}

NetworkServer::~NetworkServer()
{
    mServerStop = true;
    mpServerThread->detach();
}
} // namespace Mogwai
