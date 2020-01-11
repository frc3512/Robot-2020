// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include "communications/PublishNode.hpp"

#include <wpi/SmallVector.h>

using namespace frc3512;

PublishNode::PublishNode(std::string_view nodeName) {
    m_nodeName = nodeName;
    m_thread = std::thread(&PublishNode::RunFramework, this);
}

PublishNode::~PublishNode() {
    m_isRunning = false;
    m_ready.notify_all();
    m_thread.join();
}

void PublishNode::Subscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it == publisher.m_subList.end()) {
        publisher.m_subList.push_back(this);
    }
}

void PublishNode::Unsubscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it != publisher.m_subList.end()) {
        publisher.m_subList.erase(it);
    }
}

bool PublishNode::GetRawButton(const HIDPacket& message, int joystick,
                               int button) {
    if (joystick == 0) {
        return message.buttons1 & (1 << (button - 1));
    } else if (joystick == 1) {
        return message.buttons2 & (1 << (button - 1));
    } else if (joystick == 2) {
        return message.buttons3 & (1 << (button - 1));
    } else if (joystick == 3) {
        return message.buttons4 & (1 << (button - 1));
    } else {
        return false;
    }
}

void PublishNode::RunFramework() {
    while (m_isRunning) {
        std::unique_lock<wpi::mutex> lock(m_mutex);

        // Waits for queue to contain messages, but does not need to wait for
        // queue to contain number of contents equal to the size of a complete
        // message due to the mutex ensuring atomic message insertions.
        m_ready.wait(lock,
                     [this] { return m_queue.size() > 0 || !m_isRunning; });

        while (m_queue.size() > 0) {
            // Pops first element out of queue for length of one whole message,
            // then pops that amount elements
            size_t msgLength = m_queue.pop_front();
            wpi::SmallVector<char, 32> message;
            for (size_t i = 0; i < msgLength; i++) {
                message.push_back(m_queue.pop_front());
            }

            DeserializeAndProcessMessage(message);
        }
    }
}
