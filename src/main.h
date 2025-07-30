#ifndef MAIN_H
#define MAIN_H

#define WIDTH 600
#define HEIGHT 800

#define JOINT_RADIUS 10
#define JOINT_COUNT 4
#define LEG_COUNT 3
#define IK_ITERATIONS 128

#define BALL_RADIUS 30
#define GRAVITY 10

#define P_DARK_BLUE (Color) {0xa3, 0xb2, 0xd2, 0xff}

typedef struct leg Leg_Element;

typedef struct joint {
    Vector2 centre_position;
    float radius;
    Leg_Element *connects_from;
    Leg_Element *connects_to;
} Joint_Element;

typedef struct leg_points {
    Vector2 top_right;
    Vector2 top_left;
    Vector2 bot_left;
    Vector2 bot_right;
} Leg_Points;

typedef struct leg {
    Rectangle shape;
    Vector2 centre_pos;
    Joint_Element *origin;
    bool selected;
    Color color;
    float rotation;
    Leg_Points leg_points;
} Leg_Element;

typedef struct ball {
    Vector2 centre_position;
    Vector2 velocity;
    Vector2 acceleration;
    float radius;
    bool hit;
} Ball;



Leg_Element make_leg_element(Joint_Element* origin, float width, float height);
Vector2 get_leg_origin(Leg_Element* l);
Joint_Element make_joint_element(Leg_Element* from, Leg_Element* to, float radius);
void select_joint(Joint_Element** joints);
void handle_leg_elements(Leg_Element** legs);
void move_leg(Leg_Element* l);
void update_joint_positions(Joint_Element** joints); 
void solve_leg_chain(Vector2 target, Joint_Element** joints, int joint_count);
void rotate_legs(Joint_Element** joints);
void update_ball(Ball* b, Leg_Element** legs, float dt);
void draw_leg_points(Leg_Element* l);

#endif

