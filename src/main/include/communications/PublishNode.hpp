// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <wpi/circular_buffer.h>
#include <wpi/condition_variable.h>

#include "communications/PublishNodeBase.hpp"

namespace frc3512 {

/**
 * A communication layer that can pass messages between others who have
 * inherited it through a publish-subscribe architecture
 */
class PublishNode : public PublishNodeBase {
public:
    /**
     * Construct a PublishNode.
     *
     * @param nodeName Name of node.
     */
    explicit PublishNode(std::string_view nodeName = "Misc");
    virtual ~PublishNode();

    /**
     * Adds this object to the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode that this instance wants to recieve
     *                  event from.
     */
    void Subscribe(PublishNode& publisher);

    /**
     * Removes this object from the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode that this instance wants to stop
     *                  recieving event from.
     */
    void Unsubscribe(PublishNode& publisher);

    /**
     * Get the button value (starting at button 1).
     *
     * The buttons are returned in a single 16 bit value with one bit
     * representing the state of each button. The appropriate button is returned
     * as a boolean value.
     *
     * @param message The message whose member variables contain deserialized
     *                data.
     * @param joystick  The joystick number.
     * @param button  The button number to be read (starting at 1).
     * @return The state of the button.
     */
    static bool GetRawButton(const HIDPacket& msg, int joystick, int button);

    /**
     * Sends a packet to every subscriber.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <class P>
    void Publish(P p);

    /**
     * Sends a packet to the object it's called on.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <class P>
    void PushMessage(P p);

private:
    static constexpr int kNodeQueueSize = 1024;

    std::string m_nodeName;
    std::vector<PublishNode*> m_subList;
    wpi::circular_buffer<char> m_queue{kNodeQueueSize};

    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};
    wpi::condition_variable m_ready;

    /**
     * Blocks the thread until the queue receives at least one set of characters
     * of a message or until the node deconstructs, then processes each message.
     */
    void RunFramework();
};

}  // namespace frc3512

#include "PublishNode.inc"
