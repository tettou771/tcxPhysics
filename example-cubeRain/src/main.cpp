#include "tcApp.h"

int main() {
    tc::WindowSettings settings;
    settings.setSize(1000, 700);
    settings.setTitle("tcxPhysics - cubeRain");

    return TC_RUN_APP(tcApp, settings);
}
