#pragma once

#include <tap/util_macros.hpp>
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "robots/standard/robot_comms.hpp"
#include "tap/control/subsystem.hpp"
#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/odometry/odometry_subsystem.hpp"

#include "cv_message.hpp"

namespace src {
class Drivers;
}
namespace communication {

using subsystems::turret::TurretSubsystem;

class BeagleBoneCommunicator : public tap::communication::serial::DJISerial {
public:
    OdometryData data;
    BeagleBoneCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(BeagleBoneCommunicator);
    virtual ~BeagleBoneCommunicator() = default;
    
    /**
     * Initializes the UART line and callback interface, UART defaults to Uart1
     * 
     * Note: Uart1 is utilized by the terminal serial; to ensure smooth operation, 
     * comment out the initialize and update calls in main
     */
    void initialize();

    /**
     * Processes messages received from the BeagleBone
     * 
     * @param completeMessage The message received by the callback handler; see dji_serial.hpp
     * for more information about the structure of the message
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    /**
     * Sends messages to the BeagleBone
     */
    void sendMessage();

    /**
     * Sends odometry data to the BeagleBone
     */
    void sendOdometryData();

    /**
     * Decodes the turret aiming data received from the BeagleBone and stores it in lastTurretData
     * 
     * @param message The message received from the BeagleBone
     * @return true if message was valid and succesfully decoded, false otherwise
     */
    bool decodeTurretData(const ReceivedSerialMessage& message);

    /**
     * @return true if a message has been received within the last 
     * BEAGLEBONE_OFFLINE_TIMEOUT_MS milliseconds, false if otherwise
     */
    bool isBeagleBoneOnline() const;

    /**
     * @return a reference to lastTurretData (the last turret aiming data received from the BeagleBone)
     */

    const TurretData& getTurretData() const;
    /**
    * @return true if the Turret was written to, sets the yaw and pitch of the turret
    */
    bool setTurret(); 

private:
    src::Drivers* drivers;
    /**For positioning the Turret*/
    TurretSubsystem* turret;

    /** Last turret aiming data received from the BeagleBone */
    TurretData lastTurretData;
    
    /** UART port to communicate with the BeagleBone on */
    static constexpr tap::communication::serial::Uart::UartPort BEAGLEBONE_UART_PORT = tap::communication::serial::Uart::UartPort::Uart1;

    /** Baud rate of the UART line */
    static constexpr uint32_t BEAGLEBONE_BAUD_RATE = 115200;

    /** Time to wait between messages from BeagleBone before declaring it is offline */
    static constexpr uint16_t BEAGLEBONE_OFFLINE_TIMEOUT_MS = 2000;

    /** Timer to track elapsed time since last message received */
    tap::arch::MilliTimeout beagleboneOfflineTimeout;
};
}   // namespace communication