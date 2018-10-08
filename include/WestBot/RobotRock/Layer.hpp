// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_LAYER_HPP_
#define WESTBOT_ROBOTROCK_LAYER_HPP_

namespace WestBot {
namespace RobotRock {

/*!
 * \brief Define a page of memory representing a base layer of 64 blocks
 *        of 32 bits.
 */
typedef struct Layer
{
    uint32_t registers[ 64 ];
} __attribute__((packed)) Layer_t;

}
}

#endif // WESTBOT_ROBOTROCK_LAYER_HPP_
