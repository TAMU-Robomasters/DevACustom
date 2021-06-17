#include "referee/ref_protocol.hpp"
#include "cmsis_os.h"
#include "information/device.hpp"
#include "usart.h"
#include <stdlib.h>

userREF::SerialRxState currState;
userREF::SerialMessage watchNewMessage;

int refWatchVar = 0;

volatile uint8_t uart7InBuffer[1];

uint16_t watchCurrHp = 0;
uint16_t watchMaxHp = 0;
userREF::RobotData watchRobotData;

uint16_t frameCurrReadByte;
uint8_t frameHeader[userREF::FRAME_HEADER_LENGTH];
bool rxCrcEnabled = false;
uint16_t currCrc16 = 0;

namespace userREF {

SerialRxState djiSerialRxState;
SerialMessage newMessage;
SerialMessage mostRecentMessage;

GameData gameData;
RobotData robotData;

void task() {

    for (;;) {
        getRefereeMessage();
        watchRobotData = robotData;
        watchCurrHp = robotData.currentHp;
        watchMaxHp = robotData.maxHp;
        osDelay(1);
    }
}

void getRefereeMessage() {
    switch (djiSerialRxState) {

    case SERIAL_HEADER_SEARCH: {
        uint8_t serialHeadCheck = 0;
        while (djiSerialRxState == SERIAL_HEADER_SEARCH && HAL_UART_Receive(&huart7, &serialHeadCheck, 1, 1)) {
            //serialHeadCheck = uart7InBuffer[0];
            if (serialHeadCheck == SERIAL_HEAD_BYTE) {
                frameHeader[0] = SERIAL_HEAD_BYTE;
                newMessage.headByte = SERIAL_HEAD_BYTE;
                djiSerialRxState = PROCESS_FRAME_HEADER;
                refWatchVar = 69;
            }
        }
        watchNewMessage = newMessage;
        currState = djiSerialRxState;
        break;
    }

    case PROCESS_FRAME_HEADER: { // the frame header consists of the length, type, and CRC8
        // Read from the buffer. Keep track of the index in the frameHeader array using the
        // frameCurrReadByte. +1 at beginning and -1 on the end since the serial head
        // byte is part of the frame but has already been processed.
        frameCurrReadByte += HAL_UART_Receive(&huart7, frameHeader + frameCurrReadByte + 1, FRAME_HEADER_LENGTH - frameCurrReadByte - 1, 1);
        if (frameCurrReadByte == FRAME_HEADER_LENGTH - 1) {
						refWatchVar = frameCurrReadByte;
            frameCurrReadByte = 0;

            // process length
            newMessage.length = (frameHeader[FRAME_DATA_LENGTH_OFFSET + 1] << 8) | frameHeader[FRAME_DATA_LENGTH_OFFSET];

            // process sequence number
            newMessage.sequenceNumber = frameHeader[FRAME_SEQUENCENUM_OFFSET];
            newMessage.type = (frameHeader[FRAME_TYPE_OFFSET + 1] << 8) | frameHeader[FRAME_TYPE_OFFSET];

            if (newMessage.length == 0 || newMessage.length >= SERIAL_RX_BUFF_SIZE - (FRAME_HEADER_LENGTH + FRAME_CRC16_LENGTH)) {
                djiSerialRxState = SERIAL_HEADER_SEARCH;
                // return;
                // invalid message length check
            }

            // check crc8 on header
            if (rxCrcEnabled) {
                // uint8_t CRC8 = frameHeader[FRAME_CRC8_OFFSET];
                // don't look at crc8 or frame type when calculating crc8
                // if (!verifyCRC8(frameHeader, FRAME_HEADER_LENGTH - 3, CRC8)) {
                //     djiSerialRxState = SERIAL_HEADER_SEARCH;
                // return;
                // }
            }

            // currCrc16 = algorithms::calculateCRC16(frameHeader, FRAME_HEADER_LENGTH);

            djiSerialRxState = PROCESS_FRAME_DATA;
        }
        break;
    }

    case PROCESS_FRAME_DATA: {
        if (rxCrcEnabled) {
            // some stuff
        } else {
            frameCurrReadByte += HAL_UART_Receive(&huart7, newMessage.data + frameCurrReadByte, newMessage.length - frameCurrReadByte, 1);
        }

        if ((frameCurrReadByte == newMessage.length && !rxCrcEnabled) || (frameCurrReadByte == newMessage.length + 2 && rxCrcEnabled)) {
            frameCurrReadByte = 0;
            if (rxCrcEnabled) {
                // some stuff
            }

            // update the time and copy over the message to the most recent message
            newMessage.messageTimestamp = HAL_GetTick();

            mostRecentMessage = newMessage;

            processRefereeMessage(mostRecentMessage);

            djiSerialRxState = SERIAL_HEADER_SEARCH;

        } else if ((frameCurrReadByte > newMessage.length && !rxCrcEnabled) || (frameCurrReadByte > newMessage.length + 2 && rxCrcEnabled)) {
            frameCurrReadByte = 0;
            // invalid message length
            djiSerialRxState = SERIAL_HEADER_SEARCH;
        }
        break;
    }
    }
}

void processRefereeMessage(const SerialMessage& completeMessage) {

    switch (completeMessage.type) {
    case REF_MESSAGE_TYPE_GAME_STATUS: {
        decodeToGameStatus(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_GAME_RESULT: {
        decodeToGameResult(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_ALL_ROBOT_HP: {
        decodeToAllRobotHP(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_ROBOT_STATUS: {
        decodeToRobotStatus(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_POWER_AND_HEAT: {
        decodeToPowerAndHeat(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_ROBOT_POSITION: {
        decodeToRobotPosition(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_RECEIVE_DAMAGE: {
        decodeToReceiveDamage(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH: {
        decodeToProjectileLaunch(completeMessage);
        break;
    }
    case REF_MESSAGE_TYPE_BULLETS_REMAIN: {
        decodeToBulletsRemain(completeMessage);
        break;
    }
    default:
        break;
    }
}

// static uint16_t getRobotClientID(uint16_t robotId) { return 0x100 + robotId; }

const RobotData& getRobotData() { return robotData; }

const GameData& getGameData() { return gameData; }

bool decodeToGameStatus(const SerialMessage& message) {
    if (message.length != 11) {
        return false;
    }
    // Ignore competition type, bits [0-3] of the first byte
    gameData.gameStage = static_cast<GameStages>(message.data[0] >> 4);
    convertFromLittleEndian(&gameData.stageTimeRemaining, message.data + 1);
    // Ignore Unix time sent
    return true;
}

bool decodeToGameResult(const SerialMessage& message) {
    if (message.length != 1) {
        return false;
    }
    gameData.gameWinner = static_cast<GameWinner>(message.data[0]);
    return true;
}

bool decodeToAllRobotHP(const SerialMessage& message) {
    if (message.length != 28) {
        return false;
    }
    convertFromLittleEndian(&robotData.allRobotHp.redHero, message.data);
    convertFromLittleEndian(&robotData.allRobotHp.redEngineer, message.data + 2);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier1, message.data + 4);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier2, message.data + 6);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier3, message.data + 8);
    convertFromLittleEndian(&robotData.allRobotHp.redSentinel, message.data + 10);
    convertFromLittleEndian(&robotData.allRobotHp.redBase, message.data + 12);
    convertFromLittleEndian(&robotData.allRobotHp.blueHero, message.data + 14);
    convertFromLittleEndian(&robotData.allRobotHp.blueEngineer, message.data + 16);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier1, message.data + 18);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier2, message.data + 20);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier3, message.data + 22);
    convertFromLittleEndian(&robotData.allRobotHp.blueSentinel, message.data + 24);
    convertFromLittleEndian(&robotData.allRobotHp.blueBase, message.data + 26);
    return true;
}

bool decodeToRobotStatus(const SerialMessage& message) {
    if (message.length != 27) {
        return false;
    }
    robotData.robotId = static_cast<RobotId>(message.data[0]);
    robotData.robotLevel = message.data[1];
    convertFromLittleEndian(&robotData.currentHp, message.data + 2);
    convertFromLittleEndian(&robotData.maxHp, message.data + 4);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID1, message.data + 6);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID1, message.data + 8);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID2, message.data + 14);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID2, message.data + 16);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate42, message.data + 18);
    convertFromLittleEndian(&robotData.turret.heatLimit42, message.data + 20);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit42, message.data + 22);
    convertFromLittleEndian(&robotData.chassis.powerConsumptionLimit, message.data + 24);
    robotData.gimbalHasPower = message.data[26];
    robotData.chassisHasPower = (message.data[26] >> 1);
    robotData.shooterHasPower = (message.data[26] >> 2);

    // processReceivedDamage(clock::getTimeMilliseconds(), robotData.previousHp - robotData.currentHp);
    // robotData.previousHp = robotData.currentHp;

    return true;
}

bool decodeToPowerAndHeat(const SerialMessage& message) {
    if (message.length != 16) {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.volt, message.data);
    convertFromLittleEndian(&robotData.chassis.current, message.data + 2);
    convertFromLittleEndian(&robotData.chassis.power, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.powerBuffer, message.data + 8);
    convertFromLittleEndian(&robotData.turret.heat17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heat17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heat42, message.data + 14);
    return true;
}

bool decodeToRobotPosition(const SerialMessage& message) {
    if (message.length != 16) {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.x, message.data);
    convertFromLittleEndian(&robotData.chassis.y, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.z, message.data + 8);
    convertFromLittleEndian(&robotData.turret.yaw, message.data + 12);
    return true;
}

bool decodeToReceiveDamage(const SerialMessage& message) {
    if (message.length != 1) {
        return false;
    }
    robotData.damagedArmorId = static_cast<ArmorId>(message.data[0]);
    robotData.damageType = static_cast<DamageType>(message.data[0] >> 4);
    return true;
}

bool decodeToProjectileLaunch(const SerialMessage& message) {
    if (message.length != 7) {
        return false;
    }
    robotData.turret.bulletType = static_cast<BulletType>(message.data[0]);
    robotData.turret.launchMechanismID = static_cast<MechanismID>(message.data[1]);
    robotData.turret.firing_freq = message.data[2];
    convertFromLittleEndian(&robotData.turret.bulletSpeed, message.data + 3);
    return true;
}

bool decodeToBulletsRemain(const SerialMessage& message) {
    if (message.length != 6) {
        return false;
    }
    convertFromLittleEndian(&robotData.turret.bulletsRemaining17, message.data);
    convertFromLittleEndian(&robotData.turret.bulletsRemaining42, message.data + 2);
    convertFromLittleEndian(&robotData.remainingCoins, message.data + 4);
    return true;
}

}; // namespace userREF
