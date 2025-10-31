#ifndef NETWORKDESIGN_H
#define NETWORKDESIGN_H

#include <kilombo.h>    // <-- this will include kilolib.h on AVR, simulator API elsewhere


#define MAX_CONNECTIONS 6 //change this depending on how complex the system although 
#define CONNECTION_TIMEOUT 48  // ~3 seconds at 32 Hz

// --- Bot Types ---
typedef enum { TYPE_ALPHA = 0, TYPE_BETA, TYPE_GAMMA, TYPE_DELTA } BotType;

// --- Motion States ---
typedef enum { FORWARD, STOP, LEFT, RIGHT } motion_t;

typedef enum { FOLLOW_HOLD, FOLLOW_CHASE } follow_mode_t;

typedef enum { LEADER_EXPLORE, LEADER_WAIT } lead_mode_t;

typedef enum {
    BOT_SINGLE,    // no connections
    BOT_LEADER,    // has connections, and I’m the leader
    BOT_FOLLOWER   // has connections, but someone else is my leader
} BotRole_t;

typedef enum { WAIT, GO } walk_gate_t;

// --- Message Payload for Communication ---
typedef struct {
    uint8_t  type;        // 1
    uint8_t  id;          // 1
    uint8_t gradient;    // 1
    uint8_t  leader_id;   // 1
    uint8_t  flags;       // 1  ← new: bit0=botrole(0/1/2), bit1=WG


} __attribute__((packed)) MessagePayload;

_Static_assert(sizeof(MessagePayload) <= 9,
               "MessagePayload too large for Kilobot radio buffer");

typedef struct {
    uint8_t id;
    BotType type;
    uint8_t gradient;
    uint8_t  leader_id;
    uint8_t botrole; 
    uint8_t age;
    uint8_t dist;
    uint8_t WG;
} Connection;

// --- Per-Bot Persistent Data ---
typedef struct {
    // Movement
    BotType type;
    uint8_t id;
    uint32_t last_motion_time;
    motion_t motion_state;
    uint8_t move_interval;
    BotRole_t botrole;
    uint8_t received_botrole; // ← track incoming role

    

    // Messaging
    message_t tx_message;
    MessagePayload payload;
    uint8_t new_message;
    distance_measurement_t dist;
    uint8_t received_type;
    uint8_t received_conn;
    uint8_t received_id;

    /* trying more efficient way 
    uint8_t connections_to_type[4];  // Index 0=ALPHA, 1=BETA, etc.
    uint8_t connected_ids[MAX_CONNECTIONS];
    uint8_t connection_ages[MAX_CONNECTIONS];  // age in ticks
    */
    Connection connections[MAX_CONNECTIONS];



    // Following 
    walk_gate_t WG;
    walk_gate_t prev_WG;
    walk_gate_t myWG;
    uint8_t gradient;
    uint8_t received_gradient; 
    uint8_t  leader_id;
    uint8_t  received_leader_id; 
    float Vmin;
    uint8_t badsteps;
    uint8_t changeTicks;
    lead_mode_t lead_mode; 
    follow_mode_t follow_mode;
  
    uint8_t leader_conflict_count;




} USERDATA;

#endif // NETWORKDESIGN_H
