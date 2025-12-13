#ifndef _UTILS_H_
#define _UTILS_H_

static bool exit_app = false;

// Handle the CTRL-C keyboard signal
#ifdef _WIN32
    #include <Windows.h>

inline void CtrlHandler(DWORD fdwCtrlType) {
    exit_app = (fdwCtrlType == CTRL_C_EVENT);
}
#else
    #include <signal.h>
inline void nix_exit_handler(int s) {
    exit_app = true;
}
#endif

// Set the function to handle the CTRL-C
inline void SetCtrlHandler() {
#ifdef _WIN32
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);
#else // unix
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
#endif
}

#endif 