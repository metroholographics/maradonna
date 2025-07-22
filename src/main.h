#ifndef MAIN_H
#define MAIN_H

#define WIDTH 600
#define HEIGHT 800

#define JOINT_RADIUS 10
#define JOINT_COUNT 4

#define P_DARK_BLUE (Color) {0xa3, 0xb2, 0xd2, 0xff}

typedef struct leg Leg_Element;

typedef struct joint {
    Vector2 centre_position;
    float radius;
    Leg_Element *connects_from;
    Leg_Element *connects_to;
} Joint_Element;

typedef struct leg {
    Rectangle shape;
    Vector2 centre_pos;
    Joint_Element *origin;
    bool selected;
    Color color;
    float rotation;
} Leg_Element;

Leg_Element make_leg_element(Joint_Element* origin, float width, float height);
Vector2 get_leg_origin(Leg_Element* l);
Joint_Element make_joint_element(Leg_Element* from, Leg_Element* to, float radius);
void select_joint(Joint_Element** joints);
void handle_leg_elements(Leg_Element** legs);
void move_leg(Leg_Element* l);
void update_joint_positions(Joint_Element** joints); 
float vec2_angle(Vector2 v);
Vector2 end_joint_follow(Leg_Element *l, float x, float y);
Vector2 origin_joint_update(Leg_Element *l, Vector2 end_pos);
void handle_joint_positions(Joint_Element** joints);

#endif
