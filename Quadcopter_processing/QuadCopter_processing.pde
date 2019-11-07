/**
 * Keyboard. 
 * 
 * Click on the image to give it focus and press the letter keys 
 * to create forms in time and space. Each key has a unique identifying 
 * number. These numbers can be used to position shapes in space. 
 */

import processing.serial.*;

public class PID
{
  float kp;
  float ki;
  float kd;
};

Serial myPort;  // Create object from Serial class
int i, SerialInitVal = 0, SerialInitFlag = 0;
char axis = 'r';
String str = "ROLL";
int[] IntBuff = new int[48];
PID ROLL = new PID(), PITCH = new PID(), YAW = new PID(), ALTITUDE = new PID(), target = new PID();
int START = 0, TUNING = 0, flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0, flag5 = 0, flag6 = 0, flag7 = 0, flag8 = 0, flag9 = 0, flag10 = 0, flag11 = 0, flag12 = 0, flag13 = 0, flag14 = 0, flag15 = 0, flag16 = 0, flag17 = 0, flag18 = 0, flag19 = 0, flag20 = 0;

void setup() 
{
  size(640, 360);
  noStroke();
  background(0);

  textSize(20);

  printArray(Serial.list());

  clear();
  for (i=0; i<(Serial.list().length); i++)
  {
    text(i, 0, ((i+1) * 20));
    text(Serial.list()[i], 30, ((i+1) * 20));
  }

  text("ENTER the INDEX of the COM port to be connected ...", 0, ((i+2) * 20));
}

void draw() 
{ 
  if (SerialInitFlag == 1)
  {
    if (SerialInitVal < Serial.list().length)
    {
      SerialInitFlag = 2;
      String portName = Serial.list()[SerialInitVal];
      myPort = new Serial(this, portName, 115200);
      clear();
      text("CONNECTED to [" + Serial.list()[SerialInitVal] + "]", 0, 20);
    } else
    {
      SerialInitFlag = 0;
    }
  } else if (SerialInitFlag == 2)
  {
    if (TUNING == 1)
    {
      switch(axis)
      {
      case 'r':
        str = "ROLL";
        target = ROLL;
        break;
      case 'p':
        str = "PITCH";
        target = PITCH;
        break;
      case 'y':
        str = "YAW";
        target = YAW;
        break;
      case 'h':
        str = "ALTITUDE";
        target = ALTITUDE;
        break;
      default:
        break;
      }

      clear();
      text(str, 0, 20);
      text("KP: ", 0, 40);
      text(target.kp, 30, 40);
      text("KI: ", 0, 60);
      text(target.ki, 30, 60);
      text("KD: ", 0, 80);
      text(target.kd, 30, 80);

      println(str, ": Kp = ", target.kp, "| Ki = ", target.ki, "| Kd = ", target.kd);
      println("UP:", flag7, "| DOWN:", flag8, "| RIGHT:", flag9, "| LEFT:", flag10, "| +:", flag11, "| -:", flag12);
    }
  } else
  {
    /* Do nothing */
  }
}

void keyPressed() 
{
  int ii;

  if (SerialInitFlag == 0)
  {
    SerialInitFlag = 1;
    if (key == '0')
    {
      SerialInitVal = 0;
    } else if (key == '1')
    {
      SerialInitVal = 1;
    } else if (key == '2')
    {
      SerialInitVal = 2;
    } else if (key == '3')
    {
      SerialInitVal = 3;
    } else if (key == '4')
    {
      SerialInitVal = 4;
    } else if (key == '5')
    {
      SerialInitVal = 5;
    } else if (key == '6')
    {
      SerialInitVal = 6;
    } else if (key == '7')
    {
      SerialInitVal = 7;
    } else if (key == '8')
    {
      SerialInitVal = 8;
    } else if (key == '9')
    {
      SerialInitVal = 9;
    } else
    {
      SerialInitFlag = 0;
    }
  } else if (SerialInitFlag == 2)
  {
    if (keyCode == ENTER)
    {
      if (flag17 == 0)
      {
        flag17 = 1;

        if ((START == 0) && (TUNING == 0))
        {
          START = 1;
          myPort.write(0xFF);
          println("QuadCopter START ...");

          clear();
          text("QuadCopter START ...", 0, 20);
        }
      }
    }
    if (key == '0')
    {
      if (flag18 == 0)
      {
        flag18 = 1;

        if ((START == 1) && (TUNING == 0))
        {
          START = 0;
          myPort.write(0x00);
          println("QuadCopter STOP ...");

          clear();
          text("QuadCopter STOP ...", 0, 20);
        }
      }
    }

    if (key == '*')
    {
      if (flag19 == 0)
      {
        flag19 = 1;

        if ((TUNING == 0) && (START == 0))
        {
          TUNING = 1;
          START = 1;

          while (myPort.read() != -1)
          {
          }

          myPort.write('*');

          for (ii=0; ii<48; ii++)
          {
            IntBuff[ii] = myPort.read();
            if (IntBuff[ii] == -1)
            {
              ii--;
            }
          }

          ROLL.kp = ((IntBuff[3] << 24) | (IntBuff[2] << 16) | (IntBuff[1] << 8) | (IntBuff[0] << 0)) / 1000.0f;
          ROLL.ki = ((IntBuff[7] << 24) | (IntBuff[6] << 16) | (IntBuff[5] << 8) | (IntBuff[4] << 0)) / 1000.0f;
          ROLL.kd = ((IntBuff[11] << 24) | (IntBuff[10] << 16) | (IntBuff[9] << 8) | (IntBuff[8] << 0)) / 1000.0f;

          PITCH.kp = ((IntBuff[15] << 24) | (IntBuff[14] << 16) | (IntBuff[13] << 8) | (IntBuff[12] << 0)) / 1000.0f;
          PITCH.ki = ((IntBuff[19] << 24) | (IntBuff[18] << 16) | (IntBuff[17] << 8) | (IntBuff[16] << 0)) / 1000.0f;
          PITCH.kd = ((IntBuff[23] << 24) | (IntBuff[22] << 16) | (IntBuff[21] << 8) | (IntBuff[20] << 0)) / 1000.0f;

          YAW.kp = ((IntBuff[27] << 24) | (IntBuff[26] << 16) | (IntBuff[25] << 8) | (IntBuff[24] << 0)) / 1000.0f;
          YAW.ki = ((IntBuff[31] << 24) | (IntBuff[30] << 16) | (IntBuff[29] << 8) | (IntBuff[28] << 0)) / 1000.0f;
          YAW.kd = ((IntBuff[35] << 24) | (IntBuff[34] << 16) | (IntBuff[33] << 8) | (IntBuff[32] << 0)) / 1000.0f;

          ALTITUDE.kp = ((IntBuff[39] << 24) | (IntBuff[38] << 16) | (IntBuff[37] << 8) | (IntBuff[36] << 0)) / 1000.0f;
          ALTITUDE.ki = ((IntBuff[43] << 24) | (IntBuff[42] << 16) | (IntBuff[41] << 8) | (IntBuff[40] << 0)) / 1000.0f;
          ALTITUDE.kd = ((IntBuff[47] << 24) | (IntBuff[46] << 16) | (IntBuff[45] << 8) | (IntBuff[44] << 0)) / 1000.0f;

          println("PID TUNING STARTED ...");

          clear();
          text("PID TUNING STARTED ...", 0, 20);
        }
      }
    }
    if (key == '/')
    {
      if (flag20 == 0)
      {
        flag20 = 1;

        if ((TUNING == 1) && (START == 1))
        {
          TUNING = 0;
          START = 0;
          myPort.write('/');
          println("PID TUNING ENDED ...");

          clear();
          text("PID TUNING ENDED ...", 0, 20);
        }
      }
    }

    if (TUNING == 1)
    {
      if (key == 'r')
      {
        if (flag13 == 0)
        {
          flag13 = 1;
          axis = 'r';
          myPort.write('r');
        }
      }
      if (key == 'p')
      {
        if (flag14 == 0)
        {
          flag14 = 1;
          axis = 'p';
          myPort.write('p');
        }
      }
      if (key == 'y')
      {
        if (flag15 == 0)
        {
          flag15 = 1;
          axis = 'y';
          myPort.write('y');
        }
      }
      if (key == 'h')
      {
        if (flag16 == 0)
        {
          flag16 = 1;
          axis = 'h';
          myPort.write('h');
        }
      }

      if (key == '4')
      {
        if (flag1 == 0)
        {
          flag1 = 1;
          myPort.write('4');
          switch(axis)
          {
          case 'r':
            ROLL.kp += 0.1f;
            break;
          case 'p':
            PITCH.kp += 0.1f;
            break;
          case 'y':
            YAW.kp += 0.1f;
            break;
          case 'h':
            ALTITUDE.kp += 0.1f;
            break;
          default:
            break;
          }
        }
      } 
      if (key == '1')
      {
        if (flag2 == 0)
        {
          flag2 = 1;
          myPort.write('1');
          switch(axis)
          {
          case 'r':
            ROLL.kp -= 0.1f;
            break;
          case 'p':
            PITCH.kp -= 0.1f;
            break;
          case 'y':
            YAW.kp -= 0.1f;
            break;
          case 'h':
            ALTITUDE.kp -= 0.1f;
            break;
          default:
            break;
          }
        }
      } 
      if (key == '5')
      {
        if (flag3 == 0)
        {
          flag3 = 1;
          myPort.write('5');
          switch(axis)
          {
          case 'r':
            ROLL.ki += 0.001f;
            break;
          case 'p':
            PITCH.ki += 0.001f;
            break;
          case 'y':
            YAW.ki += 0.001f;
            break;
          case 'h':
            ALTITUDE.ki += 0.001f;
            break;
          default:
            break;
          }
        }
      } 
      if (key == '2')
      {
        if (flag4 == 0)
        {
          flag4 = 1;
          myPort.write('2');
          switch(axis)
          {
          case 'r':
            ROLL.ki -= 0.001f;
            break;
          case 'p':
            PITCH.ki -= 0.001f;
            break;
          case 'y':
            YAW.ki -= 0.001f;
            break;
          case 'h':
            ALTITUDE.ki -= 0.001f;
            break;
          default:
            break;
          }
        }
      } 
      if (key == '6')
      {
        if (flag5 == 0)
        {
          flag5 = 1;
          myPort.write('6');
          switch(axis)
          {
          case 'r':
            ROLL.kd += 0.1f;
            break;
          case 'p':
            PITCH.kd += 0.1f;
            break;
          case 'y':
            YAW.kd += 0.1f;
            break;
          case 'h':
            ALTITUDE.kd += 0.1f;
            break;
          default:
            break;
          }
        }
      } 
      if (key == '3')
      {
        if (flag6 == 0)
        {
          flag6 = 1;
          myPort.write('3');
          switch(axis)
          {
          case 'r':
            ROLL.kd -= 0.1f;
            break;
          case 'p':
            PITCH.kd -= 0.1f;
            break;
          case 'y':
            YAW.kd -= 0.1f;
            break;
          case 'h':
            ALTITUDE.kd -= 0.1f;
            break;
          default:
            break;
          }
        }
      }
    }

    if (START == 1)
    {
      if (keyCode == UP)
      {
        if (flag7 == 0)
        {
          flag7 = 1;
          myPort.write(0x01);
        }
      }
      if (keyCode == DOWN)
      {
        if (flag8 == 0)
        {
          flag8 = 1;
          myPort.write(0x02);
        }
      }
      if (keyCode == RIGHT)
      {
        if (flag9 == 0)
        {
          flag9 = 1;
          myPort.write(0x04);
        }
      }
      if (keyCode == LEFT)
      {
        if (flag10 == 0)
        {
          flag10 = 1;
          myPort.write(0x05);
        }
      }

      if (key == 'w')
      {
        if (flag7 == 0)
        {
          flag7 = 1;
          myPort.write('w');
        }
      }
      if (key == 's')
      {
        if (flag8 == 0)
        {
          flag8 = 1;
          myPort.write('s');
        }
      }
      if (key == 'd')
      {
        if (flag9 == 0)
        {
          flag9 = 1;
          myPort.write('d');
        }
      }
      if (key == 'a')
      {
        if (flag10 == 0)
        {
          flag10 = 1;
          myPort.write('a');
        }
      }

      if (key == '+')
      {
        if (flag11 == 0)
        {
          flag11 = 1;
          myPort.write('+');
        }
      } 
      if (key == '-')
      {
        if (flag12 == 0)
        {
          flag12 = 1;
          myPort.write('-');
        }
      }
    }
  }
}

void keyReleased() 
{
  if (keyCode == ENTER)
  {
    flag17 = 0;
  }
  if (key == '0')
  {
    flag18 = 0;
  }

  if (key == '*')
  {
    flag19 = 0;
  }
  if (key == '/')
  {
    flag20 = 0;
  }

  if (key == 'r')
  {
    flag13 = 0;
  }
  if (key == 'p')
  {
    flag14 = 0;
  }
  if (key == 'y')
  {
    flag15 = 0;
  }
  if (key == 'h')
  {
    flag16 = 0;
  }

  if (key == '4')
  {
    flag1 = 0;
  } 
  if (key == '1')
  {
    flag2 = 0;
  } 
  if (key == '5')
  {
    flag3 = 0;
  } 
  if (key == '2')
  {
    flag4 = 0;
  } 
  if (key == '6')
  {
    flag5 = 0;
  } 
  if (key == '3')
  {
    flag6 = 0;
  }

  if (keyCode == UP)
  {
    flag7 = 0;
    myPort.write(0x03);
  }
  if (keyCode == DOWN)
  {
    flag8 = 0;
    myPort.write(0x03);
  }
  if (keyCode == RIGHT)
  {
    flag9 = 0;
    myPort.write(0x06);
  }
  if (keyCode == LEFT)
  {
    flag10 = 0;
    myPort.write(0x06);
  }

  if (key == '+')
  {
    flag11 = 0;
  } 
  if (key == '-')
  {
    flag12 = 0;
  }
}
