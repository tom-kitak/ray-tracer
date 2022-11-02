#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <vector>

void bloom(Screen& screen, const Features& features, glm::ivec2 windowRes, float t, int distance);
