#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER  1010
#define ACK_TIMER   264
#define MAX_SEQ     127
#define INC(s)      do { s = (s + 1) % (MAX_SEQ + 1); } while(0)

typedef enum { false, true } boolean;

struct FRAME {
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned int  padding;
};

// as a sender
static unsigned char frame_nr = 0, buffer[MAX_SEQ + 1][PKT_LEN], nbuffered = 0;
static unsigned char ack_expected = 0;
static boolean phl_ready = false;

// as a receiver
static unsigned char frame_expected = 0;

static int between(unsigned char a, unsigned char b, unsigned char c)
{
    return (((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)));
}

static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

static void framing(unsigned char kind)
{
    struct FRAME s;

    s.kind = kind;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    if (kind == FRAME_DATA) {
        s.seq = frame_nr;
        memcpy(s.data, buffer[frame_nr % (MAX_SEQ + 1)], PKT_LEN);
        dbg_frame("Send DATA %3d %3d, ID %d, Window %3d, PHL Queue %d\n",
            s.seq, s.ack, *(short*)s.data, (s.seq + MAX_SEQ + 1 - ack_expected) % (MAX_SEQ + 1), phl_sq_len());
        put_frame((unsigned char*)&s, 3 + PKT_LEN);
        start_timer(frame_nr, DATA_TIMER);
        INC(frame_nr);
    }
    else {
        dbg_frame("Send ACK  %3d\n", s.ack);
        put_frame((unsigned char*)&s, 2);
    }

    stop_ack_timer();
}

int main(int argc, char** argv)
{
    int event, arg;
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);
    lprintf("Designed by Chen ZiRong, build: " __DATE__"  "__TIME__"\n");
    lprintf("Protocol Options: Piggybacking (Enabled), NAK (Disabled)\n");

    disable_network_layer();

    for (;;) {
        event = wait_for_event(&arg);

        switch (event) {
        case NETWORK_LAYER_READY:
            get_packet(buffer[frame_nr]);
            nbuffered++;
            framing(FRAME_DATA);
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = true;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char*)&f, sizeof f);
            if (len < 5 || crc32((unsigned char*)&f, len) != 0) {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                break;
            }

            if (f.kind == FRAME_ACK)
                dbg_frame("Recv ACK  %d\n", f.ack);

            if (f.kind == FRAME_DATA) {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
                if (f.seq == frame_expected) {
                    put_packet(f.data, len - 7);
                    INC(frame_expected);
                }
                start_ack_timer(ACK_TIMER);
            }

            while (between(ack_expected, f.ack, (ack_expected + nbuffered) % (MAX_SEQ + 1))) {
                stop_timer(ack_expected);
                nbuffered--;
                INC(ack_expected);   // slide window
                if (frame_nr != (ack_expected + nbuffered) % (MAX_SEQ + 1)
                    && !between(ack_expected, frame_nr, (ack_expected + nbuffered) % (MAX_SEQ + 1))) // frame_nr is received but ack is lost, so stop resending
                    frame_nr = ack_expected;
            }

            break;

        case DATA_TIMEOUT:
            dbg_event("---- DATA %d timeout\n", arg);
            frame_nr = ack_expected; // go back n
            for (int i = 0; i < nbuffered; i++)
                stop_timer((frame_nr + i) % (MAX_SEQ + 1));
            break;

        case ACK_TIMEOUT:
            framing(FRAME_ACK);
        }

        if (phl_ready && frame_nr != (ack_expected + nbuffered) % (MAX_SEQ + 1))
            framing(FRAME_DATA);  // resend

        if (nbuffered < MAX_SEQ && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}
