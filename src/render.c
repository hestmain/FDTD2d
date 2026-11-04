/* STUB - to be replaced by the owning task. */
#include "render.h"

int render_field_png(const char *path, const RenderField *field,
                     const RenderOptions *opt, RenderStats *stats, FdtdError *err)
{
    (void)path;
    (void)field;
    (void)opt;
    (void)stats;
    fdtd_error_set(err, "render_field_png: not implemented");
    return -1;
}

void render_colormap(double t, uint8_t rgb[3])
{
    (void)t;
    rgb[0] = rgb[1] = rgb[2] = 0;
}
