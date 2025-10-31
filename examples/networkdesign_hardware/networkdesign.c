#ifdef KILOBOT
  #include <kilolib.h>
  #include <avr/io.h>
#else
  #include <kilombo.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>  // for UINT16_MAX


#include "networkdesign.h"
REGISTER_USERDATA(USERDATA)

#ifdef SIMULATOR
#include <stdio.h> // for printf()
#endif

// Constants
#define BIND_RADIUS 100

#define D_MIN 45   // Min distance. Wait when this close to the one we follow
#define D_MAX 65   // Max distance. Wait when our follower is this far away
#define D_TARGET   (D_MIN + 5) // ((D_MIN + D_MAX)/2)

/// Return true if *all* of my Follower‐links satisfy `pred(dist)`.
/// If I have no Follower-links at all, returns false.
static uint8_t all_conn(uint8_t (*pred)(uint16_t)) {
    uint8_t seen_parent = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        Connection *c = &mydata->connections[i];
        if (c->id == 255)    continue;               // unused slot
        if (c->gradient != mydata->gradient + 1)  continue;  // not a “parent” link
        seen_parent = 1;
        if (!pred(c->dist)) return 0;                // one parent failed → all_conn==false
    }
    return seen_parent;  // true if at least one parent and none of them failed
}


uint8_t gt_DMAX(uint16_t d)     { return d >  D_MAX; }
uint8_t le_DMIN10(uint16_t d)   { return d <= D_MIN + 10; }

const uint8_t MOVE_INTERVAL = 64; // ~2 seconds if 32 ticks/sec

// Binding matrix
// [ALPHA, BETA, DELTA, GAMMA]
const uint8_t connection_rules[4][4] = {
    {0, 2, 0, 0}, // ALPHA connects to 2 BETA, 1 DELTA
    {2, 0, 0, 0},// BETA connects to 2 ALPHA, 1 GAMMA
    {0, 0, 2, 0}, // DELTA connects to 1 ALPHA
    {0, 0, 0, 2}  // GAMMA connects to 1 BETA
};

// Role assignment by ID
BotType assign_type(uint8_t id) {
    if (id == 0 || id == 1) return TYPE_ALPHA;
    if (id == 2 || id == 3) return TYPE_BETA;
    if (id == 4 || id == 5) return TYPE_ALPHA;
    if (id == 6 || id == 7) return TYPE_BETA;
    return TYPE_ALPHA;
}
// Smooth motor control with per-motor spin-up on hardware
void smooth_set_motors(uint8_t ccw, uint8_t cw) {
    #ifdef KILOBOT
        // Only burst the motor that’s currently off
        uint8_t spin_left  = (ccw && !OCR2A) ? 0xFF : 0;
        uint8_t spin_right = (cw  && !OCR2B) ? 0xFF : 0;
        if (spin_left || spin_right) {
            set_motors(spin_left, spin_right);
            delay(15);
        }
    #endif
        // Now set to the real target speeds (or both zero in SIM)
        set_motors(ccw, cw);
}

void set_motion(motion_t m) {
    mydata->motion_state = m;

    // (You can still add your LED feedback here if you like.)

    switch (m) {
        case LEFT:
            smooth_set_motors(kilo_turn_left, 0);
            break;
        case RIGHT:
            smooth_set_motors(0, kilo_turn_right);
            break;
        case FORWARD:
            smooth_set_motors(kilo_turn_left, kilo_turn_right);
            break;
        case STOP:
        default:
            smooth_set_motors(0, 0);
            break;
    }
}

// Setup function
void setup() {
    rand_seed(kilo_uid + kilo_ticks);

    // Movement calibration values
    kilo_turn_left = 75;
    kilo_turn_right = 75;
 
    // Follow 
    mydata->WG = GO;
    mydata->prev_WG = WAIT;
    mydata->gradient = 0;  // NO_GRADIENT
    mydata->payload.gradient = mydata->gradient;
    mydata->leader_id = kilo_uid;  // Assume self is leader initially
    mydata->payload.leader_id = mydata->leader_id;
    mydata->Vmin = 9999.0f;
    mydata->badsteps = 0;
    mydata->changeTicks = 0;
    mydata->botrole = BOT_SINGLE;
    mydata->leader_conflict_count = 0;


    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        mydata->connections[i].id = 255;  // 255 = unused
    }

    // Communication
    mydata->id = kilo_uid;
    mydata->type = assign_type(kilo_uid);
    mydata->new_message = 0;
    
    mydata->payload.type = mydata->type;
    
    mydata->payload.id = kilo_uid;

    mydata->last_motion_time = rand_soft() % 64;
    mydata->motion_state = STOP;
    mydata->move_interval = 32 + (rand_soft() % 64); // initial random interval
    mydata->lead_mode = LEADER_WAIT;

    // Bot type visualization
    switch (mydata->type) {
        case TYPE_ALPHA: set_color(RGB(3,0,0)); break;
        case TYPE_BETA:  set_color(RGB(0,0,3)); break;
        case TYPE_GAMMA: set_color(RGB(0,0,3)); break;
        case TYPE_DELTA: set_color(RGB(3,3,0)); break;
    }
}


// Can these two types form any connection (structurally valid)
uint8_t is_valid_binding(BotType t1, BotType t2) {
    return connection_rules[t1][t2] > 0 || connection_rules[t2][t1] > 0;
}

uint8_t count_connections_to_type(BotType t) {
    uint8_t count = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id != 255 && mydata->connections[i].type == t) {
            count++;
        }
    }
    return count;
}

void register_connection(uint8_t id, BotType type) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id == 255) {
            mydata->connections[i].id        = id;
            mydata->connections[i].type      = type;
            mydata->connections[i].age       = 0;
            mydata->connections[i].gradient  = mydata->received_gradient;
            mydata->connections[i].leader_id = mydata->received_leader_id;
            mydata->connections[i].dist = estimate_distance(&mydata->dist);
            mydata->connections[i].botrole = mydata->received_botrole;
            break;
        }
    }
}

// Can self bind to another bot of 'other' type, based on current counts
uint8_t can_bind(BotType self, BotType other) {
    return connection_rules[self][other] > 0 &&
           count_connections_to_type(other) < connection_rules[self][other];
}

/* // another can_bind
    uint8_t can_bind(BotType self, BotType other) {
        return mydata->connections_to_type[other] < connection_rules[self][other];
    }
*/


// Check if received_id is in our connected list
uint8_t is_connected(uint8_t id) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id == id) return 1;
    }
    return 0;
}
void update_connection_ages() {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id != 255) {
            mydata->connections[i].age++;

            if (mydata->connections[i].age > CONNECTION_TIMEOUT) {
                // Fully clear out the expired connection
                mydata->connections[i].id = 255;
                mydata->connections[i].type = 0;         
                mydata->connections[i].age = 255;
            }
        }
    }
}

uint8_t has_connections() {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id != 255) return 1;
    }
    return 0;
}

static inline uint8_t is_better(uint16_t grad1, uint8_t id1,
                                uint16_t grad2, uint8_t id2) {
    if (grad1 < grad2) {
        return 1;
    }
    if (grad1 == grad2 && id1 < id2) {
        return 1;
    }
    return 0;
}


// returns 0,1,2 for single, leader, follower
static inline BotRole_t get_bot_role(void) {
    // 1) no connections → single
    if (!has_connections()) {
        return BOT_SINGLE;
    }
    // 2) connections exist, am I the leader?
    if (mydata->leader_id == kilo_uid) {
        return BOT_LEADER;
    }
    // 3) otherwise I must be following someone else
    return BOT_FOLLOWER;
}

// This one still has some work to do. Having trouble passing on the leader role //
// when a new connection comes into contact with the current leader // 
void update_gradient() {
    // ---- Step 0: refresh my role ----
    mydata->botrole = get_bot_role();

    // ---- Case 0: single meets leader → promote yourself ----
    if (mydata->botrole == BOT_SINGLE) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_LEADER) {
                // neighbor is a leader and I'm alone → I take the crown
                mydata->botrole   = BOT_LEADER;
                mydata->leader_id = kilo_uid;
                mydata->gradient  = 0;
                return;
            }
        }
    }

    // ---- Case 1: single + single → break symmetry by UID ----
    if (mydata->botrole == BOT_SINGLE) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_SINGLE) {
                if (kilo_uid < c->id) {
                    // I become leader
                    mydata->botrole   = BOT_LEADER;
                    mydata->leader_id = kilo_uid;
                    mydata->gradient  = 0;
                } else {
                    // I become follower of them
                    mydata->botrole   = BOT_FOLLOWER;
                    mydata->leader_id = c->id;
                    mydata->gradient  = 1;
                }
                return;
            }
        }
    }

    // ---- Case 2: leader meets single → hand leadership to the new edge ----
    if (mydata->botrole == BOT_LEADER) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_SINGLE) {
                // c becomes new leader; I step back as follower
                mydata->botrole   = BOT_FOLLOWER;
                mydata->leader_id = c->id;
                mydata->gradient  = 1;
                return;
            }
        }
    }

    // ---- Case 3: single meets follower → join their chain ----
    if (mydata->botrole == BOT_SINGLE) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_FOLLOWER) {
                mydata->botrole   = BOT_FOLLOWER;
                mydata->leader_id = c->leader_id;
                mydata->gradient  = c->gradient + 1;
                return;
            }
        }
    }

    // ---- Case 4: leader meets follower of a *different* group ----
    if (mydata->botrole == BOT_LEADER) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_FOLLOWER
             && c->leader_id != mydata->leader_id) {
                // Compare leader IDs; smaller wins
                if (c->leader_id < mydata->leader_id) {
                    mydata->botrole   = BOT_FOLLOWER;
                    mydata->leader_id = c->leader_id;
                    mydata->gradient  = c->gradient + 1;
                }
                return;
            }
        }
    }

    // ---- Case 5: two leaders meet → compare by leader_id, debounced
    if (mydata->botrole == BOT_LEADER) {
        uint8_t collision = 0;  // 0 = false, 1 = true

        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;

            if (c->botrole == BOT_LEADER && c->id != kilo_uid) {
                collision = 1;

                // Only demote after 3 consecutive ticks
                if (c->id < kilo_uid) {
                    if (mydata->leader_conflict_count >= 2) {
                        // neighbor’s UID is “better”—finally step down
                        mydata->botrole        = BOT_FOLLOWER;
                        mydata->leader_id      = c->id;
                        mydata->gradient       = 1;
                        mydata->leader_conflict_count = 0;
                    } else {
                        // count this tick’s collision
                        mydata->leader_conflict_count++;
                    }
                }
                break;
            }
        }

        // no longer colliding? reset counter
        if (collision == 0) {
            mydata->leader_conflict_count = 0;
        }

        // if we saw a collision, bail out so no further cases run
        if (collision) {
            return;
        }
    }

    // ---- Case 6: two followers meet → re-align to the better group ----
    if (mydata->botrole == BOT_FOLLOWER) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->botrole == BOT_FOLLOWER
             && c->leader_id != mydata->leader_id) {
                if (c->leader_id < mydata->leader_id) {
                    mydata->leader_id = c->leader_id;
                    mydata->gradient  = c->gradient + 1;
                }
                return;
            }
        }
    }

    // ---- Default: propagate gradient down from a unified leader_id ----
    // If I’m the leader, my gradient is 0:
    if (mydata->botrole == BOT_LEADER) {
        mydata->gradient = 0;
        return;
    }
    // Otherwise pick the smallest (neighbor.gradient+1) among those
    // who share my leader_id:
    {
        uint16_t best = UINT16_MAX;
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            Connection *c = &mydata->connections[i];
            if (c->id == 255) continue;
            if (c->leader_id == mydata->leader_id) {
                uint16_t candidate = c->gradient + 1;
                if (candidate < best) best = candidate;
            }
        }
        if (best < UINT16_MAX) {
            mydata->gradient = best;
        }
    }
}

void random_walk() {
    if (kilo_ticks > mydata->last_motion_time + mydata->move_interval) {
        mydata->last_motion_time = kilo_ticks;
        mydata->move_interval = 32 + (rand_soft() % 64); // new random interval between 32–95 ticks

        uint8_t rand_dir = rand_soft() % 4;
            switch (rand_dir) {
                case 0: set_motion(FORWARD); break;
                case 1: set_motion(LEFT);    break;
                case 2: set_motion(RIGHT);   break;
                default: set_motion(STOP);   break;
            }

    }

}

// keep your existing v(), or tweak its threshold as needed:
float v(uint16_t d) {
  return (float)d - D_TARGET;
}
// new compute_potential: only consider connections one hop closer to the root
float compute_potential() {
    float V = 0.0f;

    // scan every slot in your Connection array
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        Connection *c = &mydata->connections[i];
        if (c->id == 255) continue;                // unused slot
        if (c->gradient == mydata->gradient - 1) { // parent link
            V += v(c->dist);
        }
    }

    return V;
}

// Message transmission
message_t *message_tx() {  
    /* REMOVE this line ─ it writes past the 9-byte buffer
    mydata->tx_message.data[ sizeof(MessagePayload) ] = 0;  */
    mydata->payload.gradient = mydata->gradient;
    mydata->payload.leader_id = mydata->leader_id;
    mydata->payload.flags = ((uint8_t)mydata->botrole & 0x03) | ((uint8_t)mydata->WG << 2);

    mydata->tx_message.type = NORMAL;    
    memcpy(&mydata->tx_message.data, &mydata->payload, sizeof(MessagePayload));
    mydata->tx_message.crc = message_crc(&mydata->tx_message);


    return &mydata->tx_message;
}

// Message reception
void message_rx(message_t *m, distance_measurement_t *d) {
    MessagePayload *recv = (MessagePayload *)m->data;
    mydata->received_type = recv->type;
    mydata->dist = *d;
    mydata->new_message = 1;
    mydata->received_id = recv->id;
    mydata->received_gradient = recv->gradient;
    mydata->received_leader_id = recv->leader_id;


     /* ------- unpack back into the enums -------- */
    mydata->received_botrole = (BotRole_t)(recv->flags & 0x03);
    mydata->WG               = (walk_gate_t)((recv->flags >> 2) & 0x01);


    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id == recv->id) {
            mydata->connections[i].age       = 0;
            mydata->connections[i].dist      = estimate_distance(d);
            mydata->connections[i].gradient  = recv->gradient;
            mydata->connections[i].leader_id = recv->leader_id;
            mydata->connections[i].botrole = mydata->received_botrole;
            mydata->connections[i].WG = mydata->WG;

            break;
        }
    }
}

// This one seems pretty good 
static inline void do_leader_logic(void) { 
    // Remember last walk‐gate so followers can detect the WAIT→GO transition
    walk_gate_t last_WG = mydata->WG;

    // If we’re in GO, we explore until everyone falls outside D_MAX
    if (last_WG == GO) {
        if (all_conn(gt_DMAX)) {
            // everyone’s too far: switch to WAIT
            mydata->WG = GO;
            set_color(RGB(1,1,1));   // white = waiting
            set_motion(STOP);
        } else {
            // still got followers in range: keep exploring
            set_color(RGB(0,3,3));   // cyan = exploring
            random_walk();
        }

    // If we’re in WAIT, we hold until everyone closes back in
    } else {  // last_WG == WAIT
        if (all_conn(le_DMIN10)) {
            // all back inside D_MIN+10: switch to GO
            mydata->WG = WAIT;
            set_color(RGB(0,3,3));   // cyan = exploring
            random_walk();
        } else {
            // still waiting on stragglers
            set_color(RGB(1,1,1));   // white = waiting
            set_motion(STOP);
        }
    }

    // Update prev_WG for your followers’ Vmin reset logic
    mydata->prev_WG = last_WG;
}

static inline void do_follower_logic(void) {
    float V    = compute_potential();
    float err  = fabsf(V);
    
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        Connection *c = &mydata->connections[i];
        if (c->id == 255)    continue;               // unused slot
        if (c->gradient != mydata->gradient - 1)  continue;  // not a “parent” link
        mydata->prev_WG = c->WG;
    }
    // if we’re within say ±5mm of target, just hold still SOMEWHERE ELSE
    if (err < 5.0f) {
        set_motion(STOP);
        return;
    }

    // uint8_t other = (mydata->motion_state == LEFT ? RIGHT : LEFT);

    // 1) on WAIT→GO reset:
    if (mydata->prev_WG == WAIT) {
        mydata->Vmin       = err;
        mydata->badsteps   = 0;
        mydata->changeTicks= 20;
        mydata->motion_state = STOP;
        
    } else {
        if (err < mydata->Vmin && (mydata->changeTicks <= 0 || mydata->changeTicks == 20)) {
                mydata->badsteps = 0;
                mydata->changeTicks = 20;
                mydata->motion_state = FORWARD; // arbitrary movement one or the other
        }else{
            --mydata->changeTicks;
            mydata->badsteps = 1; // use different variable
            mydata->motion_state = LEFT;
        }
        /*
        if (mydata->badsteps == 0){
            // 3) dead‐time bookkeeping
            if (mydata->changeTicks > 0) {
                if (--mydata->changeTicks == 0) {
                    if (err > mydata->Vmin) {
                        mydata->badsteps++;
                    }
                }
            }
            // 4) after dead‐time + one bad pass, flip direction
            else if (err > mydata->Vmin && mydata->badsteps < 1) {
                mydata->motion_state = other;
                mydata->changeTicks  = 20;   // dead‐time
        }
        }
        */
    }
    // 5) drive it
    mydata->Vmin = err;
    set_color(RGB(3,3,0));  
    set_motion(mydata->motion_state); 
}



// Main loop
void loop() {
    
    // Handle incoming messages 
    if (mydata->new_message) {
        mydata->new_message = 0;
        uint16_t dist_mm = estimate_distance(&mydata->dist);
        
        // 2. Ignore if structurally invalid
        if (!is_valid_binding(mydata->type, mydata->received_type)) {
            return;
        }

        
        if (!is_connected(mydata->received_id) && dist_mm < BIND_RADIUS &&
            can_bind(mydata->type, mydata->received_type)) {
            
            register_connection(mydata->received_id, mydata->received_type);
        }
        // Count active connections
        uint8_t total_connections = 0;
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (mydata->connections[i].id != 255) total_connections++;
        }
    }   

    // Debugging code
    /*
    #ifdef SIMULATOR
    printf("Tick %d | ID %d | cycle %d\n", kilo_ticks, kilo_uid, cycle);
    #endif
    */

    // Update Gradient for moving together
    update_gradient();

    // this is just a test block to show that alphas can be leaders and followers 
    if (mydata->type == TYPE_ALPHA) {
        if (mydata->gradient == 0) {
            set_color(RGB(3, 0, 0));  // bright red root
        } else {
            set_color(RGB(1, 0, 0));  // dimmer red follower
        }
    }
    // OLD movement Logic opah complex complex 
    /* About 90 lines of nada
    switch (mydata->state) {  
        case STATE_CONNECTED: {
            uint8_t is_true_leader = (mydata->gradient % 2 == 0) && (mydata->leader_id == kilo_uid);

            uint8_t valid_range = 0;
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (mydata->connections[i].id != 255 &&
                    mydata->connections[i].dist >= D_MIN &&
                    mydata->connections[i].dist <= D_MAX) {
                    valid_range = 1;
                    break;
                }
            }

            if (is_true_leader) {
                switch (mydata->lead_mode) {

                case LEADER_EXPLORE:
                    // Keep walking until **all** followers are beyond D_MAX 
                    if (all_conn(gt_DMAX)) {
                        set_motion(STOP);
                        mydata->lead_mode = LEADER_WAIT;
                    } else {
                        set_color(RGB(0,3,3));   // cyan = exploring
                        random_walk();
                    }
                    break;

                case LEADER_WAIT:
                    // Stay put until **all** followers are back inside D_MIN+10 
                    if (all_conn(le_DMIN10)) {
                        mydata->lead_mode = LEADER_EXPLORE;
                        set_color(RGB(0,3,3));
                        random_walk();           // kick-off
                    } else {
                        set_color(RGB(1,1,1));   // white = waiting
                        set_motion(STOP);
                    }
                    break;
                }
            
            } else {
                float V = compute_potential();
                uint8_t otherstate = (mydata->motion_state == LEFT) ? RIGHT : LEFT;

                // Update minimum potential
                if (mydata->prev_WG == WAIT){
                    mydata->Vmin = V;
                }
                if (V < mydata->Vmin) {
                    mydata->Vmin = V;
                    mydata->badsteps = 0;

                    // If improving, go forward
                    mydata->motion_state = FORWARD;
                    mydata->changeTicks = 0;
                } else {
                    if (mydata->changeTicks > 0) {
                        mydata->changeTicks--;
                    } else {
                        mydata->badsteps++;

                        if (mydata->badsteps >= 2) {
                            // Switch direction after enough bad steps
                            mydata->motion_state = otherstate;
                            mydata->changeTicks = 60; // cooldown before reassessing
                            mydata->badsteps = 0;
                            mydata->Vmin = V;
                        }
                    }
                }

                set_color(RGB(3, 3, 0));  // Yellow = follower
                set_motion(mydata->motion_state);
            }
            break;

        }

        case STATE_SEEKING: {
            set_color(RGB(1,0,1));  // Purple = searching
            
            // add if no neighbors then random_walk but if neighbors then edge until found valid connection
            random_walk();
            
            break;
        }   

        
    } */

    switch(mydata->botrole) {
      case BOT_SINGLE:
        // exactly what you used to do in STATE_SEEKING:
        set_color(RGB(1,0,1));    // purple = searching
        random_walk();
        break;

      case BOT_LEADER:
        // exactly what you used to do in STATE_CONNECTED when is_true_leader:
        do_leader_logic();
        break;

      case BOT_FOLLOWER:
        // exactly what you used to do in STATE_CONNECTED otherwise:
        if (mydata->WG == GO) {
            do_follower_logic();
            } else {
            set_motion(STOP);
            }
        break;
    }
    mydata->prev_WG = mydata->WG;


    // Remove stale connections/reset gradient
    update_connection_ages();
}


// Empty obstacle callback
int16_t callback_obstacles(double x, double y, double *dx, double *dy) {
    return 0;
}


#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
static char *role_str[] = { "SINGLE", "LEADER", "FOLLOWER" };

char *cb_botinfo(void)
{
    char *p = botinfo_buffer;

    // print potential
    float V = compute_potential();
    p += sprintf(p, "VMIN Potential: %.2f\n", mydata->Vmin);
     // print my current role 
    p += sprintf(p, "Role: %s\n", role_str[ mydata->botrole ]);

    // Count connections to each type
    uint8_t alpha_count = 0, beta_count = 0, delta_count = 0, gamma_count = 0;

    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id != 255) {
            switch (mydata->connections[i].type) {
                case TYPE_ALPHA: alpha_count++; break;
                case TYPE_BETA:  beta_count++;  break;
                case TYPE_DELTA: delta_count++; break;
                case TYPE_GAMMA: gamma_count++; break;
                default: break;
            }
        }
    }

    p += sprintf(p, "Type: %d\n", mydata->type);
    p += sprintf(p, "Gradient: %d\n", mydata->gradient);
    p += sprintf(p, "ID: %d\n", kilo_uid);
    p += sprintf(p, "Connections to Types: [A:%d, B:%d, D:%d, G:%d]\n",
                 alpha_count, beta_count, delta_count, gamma_count);

    // Optional: show all connected bot IDs
    p += sprintf(p, "Connected IDs: ");
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (mydata->connections[i].id != 255) {
            p += sprintf(p, "%d ", mydata->connections[i].id);
        }
    }
    p += sprintf(p, "\n");

    return botinfo_buffer;
}
#endif


// Main
int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    SET_CALLBACK(botinfo, cb_botinfo);
    kilo_start(setup, loop);
    return 0;
}