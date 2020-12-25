// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#ifdef _WIN32
#define _WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX

#include <winsock2.h>

#endif

/**
 * Emulates pipe(3).
 *
 * @param fds Array of file descriptors for the pipe where fds[0] is the pipe's
 *            read end and fds[1] is the pipe's write end.
 * @return 0 on success or -1 on failure.
 */
int pipe(SOCKET fds[2]);

#endif
