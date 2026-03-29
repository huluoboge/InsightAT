#include "render_object.h"
namespace insight {

namespace render {
bool g_exit_render = false;
RENDER_EXPORT void exitRender() { g_exit_render = true; }

RENDER_EXPORT void startRender() { g_exit_render = false; }
} // namespace render

} // namespace insight
