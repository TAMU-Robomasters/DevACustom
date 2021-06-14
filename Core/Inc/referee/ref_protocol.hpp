#pragma once
#include "information/device.hpp"

/**
 *
 * Structure of a Serial Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Frame Head Byte (0xA5)                                     |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Frame Data Length, LSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Frame Data Length, MSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 3               | Frame Sequence Number                                      |
 * +-----------------+------------------------------------------------------------+
 * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
 * +-----------------+------------------------------------------------------------+
 * | 5               | Message Type, LSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | 6               | Message Type, MSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * | Message CRC                                                                  |
 * +-----------------+------------------------------------------------------------+
 * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
 * +-----------------+------------------------------------------------------------+
 * | 8 + Data Length | CRC16 of header and frame, MSB                             |
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */

static const uint16_t SERIAL_RX_BUFF_SIZE = 256;
static const uint16_t SERIAL_TX_BUFF_SIZE = 256;
static const uint16_t SERIAL_HEAD_BYTE = 0xA5;
static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
static const uint8_t FRAME_CRC8_OFFSET = 4;
static const uint8_t FRAME_HEADER_LENGTH = 7;
static const uint8_t FRAME_TYPE_OFFSET = 5;
static const uint8_t FRAME_CRC16_LENGTH = 2;
static constexpr uint16_t SERIAL_RX_FRAME_HEADER_AND_BUF_SIZE = SERIAL_RX_BUFF_SIZE + FRAME_HEADER_LENGTH;

enum SerialRxState {
    SERIAL_HEADER_SEARCH, /// A header byte has not yet been received.
    PROCESS_FRAME_HEADER, /// A header is received and the frame header is being processed.
    PROCESS_FRAME_DATA    /// The data is being processed.
};
extern SerialRxState djiSerialRxState;

struct SerialMessage {
    uint8_t headByte; /// Use `SERIAL_HEAD_BYTE`.
    uint16_t length;  /// Must be less than SERIAL_RX_BUFF_SIZE or SERIAL_TX_BUFF_SIZE.
    uint16_t type;    /// The type is specified and interpreted by a derived class.
    uint8_t data[SERIAL_RX_BUFF_SIZE];
    uint32_t messageTimestamp; /// The timestamp is in milliseconds.
    uint8_t sequenceNumber;    /// A derived class may increment this for debugging purposes.
};
// Message in middle of being constructed.
extern SerialMessage newMessage;
// Most recent complete message.
extern SerialMessage mostRecentMessage;

enum GameStages {
    PREMATCH = 0,       /// Pre-competition. stage
    SETUP = 1,          /// Setup stage.
    INITIALIZATION = 2, /// Initialization stage.
    COUNTDOWN = 3,      /// 5-second countdown.
    IN_GAME = 4,        /// In middle of the game.
    END_GAME = 5,       /// Calculating competition results.
};

enum GameWinner {
    DRAW = 0, /// Match was a draw.
    RED = 1,  /// Red team won the match.
    BLUE = 2, /// Blue team won the match.
};

enum RobotId {
    INVALID = 0,

    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_SOLDIER_1 = 3,
    RED_SOLDIER_2 = 4,
    RED_SOLDIER_3 = 5,
    RED_DRONE = 6,
    RED_SENTINEL = 7,
    RED_DART = 8,
    RED_RADAR_STATION = 9,

    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_SOLDIER_1 = 103,
    BLUE_SOLDIER_2 = 104,
    BLUE_SOLDIER_3 = 105,
    BLUE_DRONE = 106,
    BLUE_SENTINEL = 107,
    BLUE_DART = 108,
    BLUE_RADAR_STATION = 109
};

enum ArmorId {
    FRONT = 0, /// armor #0 (front).
    LEFT = 1,  /// armor #1 (left).
    REAR = 2,  /// armor #2 (rear).
    RIGHT = 3, /// armor #3 (right).
    TOP = 4,   /// armor #4 (top).
};

enum DamageType {
    ARMOR_DAMAGE = 0,          /// Armor damage.
    MODULE_OFFLINE = 1,        /// Module offline.
    BARREL_OVER_SPEED = 2,     /// Firing speed too high.
    BARREL_OVERHEAT = 3,       /// Barrel overheat.
    CHASSIS_POWER_OVERRUN = 4, /// Chassis power overrun.
    COLLISION = 5,             /// Chassis collision.
};

enum DeleteGraphicOperation {
    DELETE_GRAPHIC_NO_OP = 0,
    DELETE_GRAPHIC_LAYER = 1,
    DELETE_ALL = 2,
};

enum AddGraphicOperation {
    ADD_GRAPHIC_NO_OP = 0,
    ADD_GRAPHIC = 1,
    ADD_GRAPHIC_MODIFY = 2,
    ADD_GRAPHIC_DELETE = 3 /// Not sure why you can specify delete when adding a graphic
};

enum GraphicType {
    STRAIGHT_LINE = 0,
    RECTANGLE = 1,
    CIRCLE = 2,
    ELLIPSE = 3,
    ARC = 4,
    FLOATING_NUM = 5,
    INTEGER = 6,
    CHARACTER = 7,
};

enum GraphicColor {
    RED_AND_BLUE = 0,
    YELLOW = 1,
    GREEN = 2,
    ORANGE = 3,
    PURPLISH_RED = 4,
    PINK = 5,
    CYAN = 6,
    BLACK = 7,
    WHITE = 8,
};

struct DamageEvent {
    uint16_t damageAmount; /// Amount of damage received
    uint32_t timestampMs;  /// Time when damage was received (in milliseconds).
};

enum BulletType {
    AMMO_17 = 1, /// 17 mm projectile ammo.
    AMMO_42 = 2, /// 42 mm projectile ammo.
};

enum MechanismID {
    TURRET_17MM_1 = 1,
    TURRET_17MM_2 = 2,
    TURRET_42MM = 3,
};

struct GameData {
    GameStages gameStage : 4;    /// Current stage in the game.
    uint16_t stageTimeRemaining; /// Remaining time in the current stage (in seconds).
    GameWinner gameWinner;       /// Results of the match.
};

struct ChassisData {
    uint16_t volt;        /// Output voltage to the chassis (in mV).
    uint16_t current;     /// Output current to the chassis (in mA).
    float power;          /// Output power to the chassis (in W).
    uint16_t powerBuffer; /// Chassis power buffer (in J).
    float x, y, z;        /// x, y, z coordinate of the chassis.
    uint16_t powerConsumptionLimit;
};

struct TurretData {
    BulletType bulletType;          /// 17mm or 42mm last projectile shot.
    MechanismID launchMechanismID;  /// Either 17mm mechanism 1, 3, or 42 mm mechanism.
    uint8_t firing_freq;            /// Firing frequency (in Hz).
    uint16_t heat17ID1;             /// Current 17mm turret heat, ID2.
    uint16_t heat17ID2;             /// ID2 turret heat.
    uint16_t heatCoolingRate17ID1;  /// 17mm turret cooling value per second, ID1.
    uint16_t heatCoolingRate17ID2;  /// ID2.
    uint16_t heatLimit17ID1;        /// 17mm turret heat limit, ID1.
    uint16_t heatLimit17ID2;        /// ID2.
    uint16_t barrelSpeedLimit17ID1; /// 17mm turret barrel speed limit, ID1.
    uint16_t barrelSpeedLimit17ID2; /// ID2.
    uint16_t heat42;                /// Current 42mm turret heat.
    uint16_t heatCoolingRate42;     /// 42mm turret cooling value per second.
    uint16_t heatLimit42;           /// 42mm turret heat limit.
    uint16_t barrelSpeedLimit42;    /// 42mm turret barrel sp   eed.
    uint16_t bulletsRemaining17;    /// Number of bullets remaining in sentinel
                                    /// and drone only (500 max) if in RMUC, or
                                    /// any robot in RMUL.
    uint16_t bulletsRemaining42;    /// Number of bullets remaining in hero if in RMUL
                                    /// or 0 if in RMUC.
    float bulletSpeed;              /// Last bullet speed (in m/s).
    float yaw;                      /// Barrel yaw position (degree).
};

struct RobotHpData {
    // current HP of all robots
    uint16_t redHero;
    uint16_t redEngineer;
    uint16_t redSoldier1;
    uint16_t redSoldier2;
    uint16_t redSoldier3;
    uint16_t redSentinel;
    uint16_t redBase;
    uint16_t blueHero;
    uint16_t blueEngineer;
    uint16_t blueSoldier1;
    uint16_t blueSoldier2;
    uint16_t blueSoldier3;
    uint16_t blueSentinel;
    uint16_t blueBase;
};

struct RobotData {
    RobotId robotId;             /// Robot type and team.
    uint8_t robotLevel;          /// Current level of this robot (1-3).
    uint16_t previousHp;         /// Health of this robot before damage was
                                 /// received, used to calculate receivedDps
                                 /// if no damage was received recently,
                                 /// previousHp = currentHp.
    uint16_t currentHp;          /// Current health of this robot.
    uint16_t maxHp;              /// Max health of this robot.
    uint8_t gimbalHasPower : 1;  /// 1 if there is 24V output to gimbal, 0 for 0V.
    uint8_t chassisHasPower : 1; /// 1 if there is 24V output to chassis, 0 for 0V.
    uint8_t shooterHasPower : 1; /// 1 if there is 24V output to shooter, 0 for 0V.
    ArmorId damagedArmorId : 4;  /// Armor ID that was damaged.
    DamageType damageType : 4;   /// Cause of damage.
    float receivedDps;           /// Damage per second received.
    ChassisData chassis;         /// Chassis power draw and position data.
    TurretData turret;           /// Turret firing and heat data.
    RobotHpData allRobotHp;      /// Current HP of all the robots.
    uint16_t remainingCoins;     /// Number of remaining coins left to spend.
};