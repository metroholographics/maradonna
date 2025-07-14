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

    foot = make_leg_element(&ankle, 75.0f, 50.0f);


    SetTargetFPS(60);
    while (!WindowShouldClose())
    {
        static float rotation = 0;
        // rotation += 0.05f;
        // if (rotation >= 360.0f) rotation = 0.0f;

        // if (IsKeyPressed(KEY_SPACE)) {
        //     rotation = 0.0f;
        // }

        BeginDrawing();
            ClearBackground(P_DARK_BLUE);
            DrawRectanglePro(thigh.shape, get_leg_origin(&thigh), rotation, RED);
            DrawRectanglePro(leg.shape, get_leg_origin(&leg), rotation, RED);
            DrawRectanglePro(foot.shape, get_leg_origin(&foot), rotation, RED);
            DrawCircleV(hip.centre_position, hip.radius, GREEN);
            DrawCircleV(knee.centre_position, knee.radius, GREEN);
            DrawCircleV(ankle.centre_position, ankle.radius, GREEN);
        EndDrawing();
    }

    CloseWindow();
}