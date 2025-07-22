#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"

Leg_Element thigh;
Leg_Element leg;
Leg_Element foot;
Joint_Element hip;
Joint_Element knee;
Joint_Element ankle;
Joint_Element toe;

Joint_Element* joints_array[JOINT_COUNT];
Leg_Element* legs_array[3];


int selected_joint;


Vector2 get_leg_origin(Leg_Element* l)
{
    return (Vector2) {
        .x = l->shape.width,
        .y = 0,
    };
}

Leg_Element make_leg_element(Joint_Element* origin, float width, float height)
{
    Leg_Element l;
    l.origin = origin;
    l.shape.width = width;
    l.shape.height = height;
    l.shape.x = l.origin->centre_position.x;
    l.shape.y = l.origin->centre_position.y;
    l.centre_pos = (Vector2) {
        .x = l.shape.x + (0.5f * l.shape.width),
        .y = l.shape.y + (0.5f * l.shape.height)
    };
    l.color = RED;
    l.selected = false;
    l.rotation = 0.0f;
    return l;
}

Joint_Element make_joint_element(Leg_Element* from, Leg_Element* to, float radius)
{
    Joint_Element j;
    j = (Joint_Element) {
        .connects_from = from,
        .connects_to = to,
        .radius = radius,
    };
    if (from != NULL) {
        j.centre_position = (Vector2) {
            .x = j.connects_from->shape.x - j.connects_from->shape.width,
            .y = j.connects_from->shape.y + j.connects_from->shape.height
        };
    }

    return j;
}

void select_joint(Joint_Element** joints)
{
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) return;

    Vector2 mouse_pos = GetMousePosition();

    for (int i = 0; i < 4; i++) {
        if (joints[i]->connects_from == NULL) continue;
        if (CheckCollisionPointCircle(mouse_pos, joints[i]->centre_position, JOINT_RADIUS)) {
            if (selected_joint != -1) {
                joints_array[selected_joint]->connects_from->selected = false;
            }
            joints[i]->connects_from->selected = true;
            selected_joint = i;
        } 
    }
}

void move_leg(Leg_Element* l) 
{
    Vector2 mouse_d = GetMouseDelta();
    //printf("x %f y %f\n", mouse_d.x, mouse_d.y);

    if (mouse_d.y > 0) {
        l->rotation -= 0.25f * mouse_d.y;
    } else if (mouse_d.y < 0) {
        l->rotation += 0.25 * -mouse_d.y;
    }
        
}

void handle_leg_elements(Leg_Element** legs)
{
    for (int i = 0; i < 3; i++) {
        if (legs[i]->selected) {
            legs[i]->color = BLUE;
        } else {
            legs[i]->color = RED;
        }
        legs[i]->shape.x = legs[i]->origin->centre_position.x;
        legs[i]->shape.y = legs[i]->origin->centre_position.y;
    }
}


float vec2_angle(Vector2 v)
{
    if (v.x == 0) return 0;
    float angle = atan(v.y / v.x);

    if (v.y < 0 && v.x < 0) angle += PI;
    else if (v.x < 0) angle += PI;
    return angle;
}

Vector2 get_rotated_end(Leg_Element l)
{
    float dx = -l.shape.width;
    float dy = l.shape.height;
    float angle = DEG2RAD * l.rotation;
    float rx = dx * cosf(angle) - dy * sinf(angle);
    float ry = dx * sinf(angle) + dy * cosf(angle);
    return (Vector2) {rx, ry};
}

Vector2 end_joint_follow(Leg_Element *l, float x, float y)
{
    Vector2 t = (Vector2) {x, y};
    Vector2 rotated_pos = get_rotated_end(*l);

    Vector2 dir = Vector2Subtract(t, rotated_pos);
    float angle = vec2_angle(dir);
    l->rotation = RAD2DEG * angle;

    rotated_pos.x = t.x - (l->shape.width)*cosf(angle);
    rotated_pos.y = t.y - (l->shape.width)*sinf(angle);
    return rotated_pos;
}

Vector2 origin_joint_update(Leg_Element *l, Vector2 end_pos)
{
    Vector2 b;
    float angle = DEG2RAD * l->rotation;
    b.x = end_pos.x - l->shape.width * cosf(angle);
    b.y = end_pos.y - l->shape.width * sinf(angle);
    return b;
}

void handle_joint_positions(Joint_Element** joints)
{
    Joint_Element* end_joint = joints[JOINT_COUNT - 1];
    Leg_Element* connected_leg = end_joint->connects_from;
    if (connected_leg == NULL) return;

    end_joint->centre_position = end_joint_follow(connected_leg, GetMouseX(), GetMouseY());

    for (int i = JOINT_COUNT-1; i >= 0; i--) 
    {
        Leg_Element* connected_leg = joints[i]->connects_from;
        if (connected_leg == NULL) break;
        Vector2 end_pos = joints[i]->centre_position;
        Vector2 c = (Vector2) {connected_leg->shape.x, connected_leg->shape.y};
        Vector2 dir = Vector2Subtract(end_pos, c);
        float angle = vec2_angle(dir);
        connected_leg->rotation = RAD2DEG * angle;
        connected_leg->origin->centre_position = origin_joint_update(connected_leg, end_pos);
    }
}

void update_joint_positions(Joint_Element** joints)
{
    for (int i = 0; i < 4; i++) 
    {
        if (i == 0) continue;
        Leg_Element parent = *joints[i]->connects_from;

        Vector2 rotated = get_rotated_end(parent);

        joints[i]->centre_position.x = parent.shape.x + rotated.x;
        joints[i]->centre_position.y = parent.shape.y + rotated.y;
        if (joints[i]->connects_to != NULL) {
            joints[i]->connects_to->shape.x = joints[i]->centre_position.x;
            joints[i]->connects_to->shape.y = joints[i]->centre_position.y;
        }
    }
}

int main (int argc, char* argv[])
{
    (void)argc; (void)argv;

    InitWindow(WIDTH, HEIGHT, "maradonna");

    hip = make_joint_element(NULL, &thigh, JOINT_RADIUS);
    hip.centre_position = (Vector2) {WIDTH, 500};

    thigh = make_leg_element(&hip, 100.0f, 50.0f);

    knee = make_joint_element(&thigh, &leg, JOINT_RADIUS);

    leg = make_leg_element(&knee, 50.0f, 150.0f);

    ankle = make_joint_element(&leg, &foot, JOINT_RADIUS);

    foot = make_leg_element(&ankle, 75.0f, 30.0f);

    toe = make_joint_element(&foot, NULL, JOINT_RADIUS);

    SetTargetFPS(60);
    selected_joint = -1;

    joints_array[0] = &hip;
    joints_array[1] = &knee;
    joints_array[2] = &ankle;
    joints_array[3] = &toe;
   
    legs_array[0] = &thigh;
    legs_array[1] = &leg;
    legs_array[2] = &foot;
    
    while (!WindowShouldClose())
    {

        select_joint(joints_array);
        // update_joint_positions(joints_array);

        
        handle_joint_positions(joints_array);
        handle_leg_elements(legs_array);

        BeginDrawing();
            ClearBackground(P_DARK_BLUE);
            DrawRectanglePro(thigh.shape, get_leg_origin(&thigh), thigh.rotation, thigh.color);
            DrawRectanglePro(leg.shape, get_leg_origin(&leg), leg.rotation, leg.color);
            DrawRectanglePro(foot.shape, get_leg_origin(&foot), foot.rotation, foot.color);
            DrawCircleV(hip.centre_position, hip.radius, GREEN);
            DrawCircleV(knee.centre_position, knee.radius, GREEN);
            DrawCircleV(ankle.centre_position, ankle.radius, GREEN);
            DrawCircleV(toe.centre_position, toe.radius, GREEN);
        EndDrawing();
    }

    CloseWindow();
}