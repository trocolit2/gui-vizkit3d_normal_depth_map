#version 130

attribute vec2 position;

void main()
{
    float pos_x = position.x;
    float pos_y = (position.y - 0.5) * 2;
    gl_Position = vec4(pos_x, pos_y, 0.0, 1.0);
}
