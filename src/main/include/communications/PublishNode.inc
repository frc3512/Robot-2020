// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

namespace frc3512 {

template <class P>
void PublishNode::Publish(P p) {
    p.topic = m_nodeName + "/" + p.topic;
    for (auto sub : m_subList) {
        sub->PushMessage(p);
    }
}

template <class P>
void PublishNode::PushMessage(P p) {
    Packet packet = p.Serialize();
    size_t len = packet.getDataSize();
    auto ptr = static_cast<const char*>(packet.getData());

    std::lock_guard lock(m_mutex);
    m_queue.push_back(len);
    for (size_t j = 0; j < len; j++) {
        m_queue.push_back(ptr[j]);
    }
    m_ready.notify_one();
}

}  // namespace frc3512
