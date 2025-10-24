#pragma once

uint16_t RGB_to_D(const RGB &color);

void D_to_RGB(const uint16_t &depth, RGB &color);

template<typename F>
void make_rgb_data(const uint16_t *depth_data, RGB *rgb_data, int width, int height, F coloring_func);

void depth_to_color(const uint16_t *depth, int h, int w, RGB *colorized_depth, const bool is_inverse);

void color_to_depth(const RGB *colorized_depth, int h, int w, uint16_t *depth, const bool is_inverse);

void view_colorized_depth(const RGB *colorized_depth, int h, int w);

int view_colorized_depth_map();

float calc_depth_error(const uint16_t *depth, const uint16_t *depth_recover, int h, int w);