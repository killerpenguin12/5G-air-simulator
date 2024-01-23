/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of 5G-air-simulator
 *
 * 5G-air-simulator is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * 5G-air-simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 5G-air-simulator; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Telematics Lab <telematics-dev@poliba.it>
 */

#include "FromInput.h"

InputPosition::InputPosition(double x, double y, int speed, double direction)
{
  SetMobilityModel(Mobility::INPUT);
  CartesianCoordinates *newPosition =
    new CartesianCoordinates(x,y);
  SetAbsolutePosition (newPosition);
  SetSpeedDirection(direction);
  SetSpeed(speed);
  delete newPosition;
}

InputPosition::~InputPosition()
{
  DeleteAbsolutePosition ();
}

void
InputPosition::UpdatePosition (double x, double y)
{
    CartesianCoordinates *newPosition =
    new CartesianCoordinates(x,y);
    SetAbsolutePosition (newPosition);
    delete newPosition;
}

void
InputPosition::UpdateSpeed (int speed, double direction)
{
    SetSpeedDirection(direction);
    SetSpeed(speed);
}
