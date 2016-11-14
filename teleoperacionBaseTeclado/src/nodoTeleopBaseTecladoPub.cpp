/*
 * teleop_base_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sys/poll.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
// La velocidad lineal maxima de la base es de 20 cm/s -> 0'200 m/s.
// 10 cm/s ==> 0'100 m/s
double g_max_speed = 0.100;
// 10 grad/s ==> (10.0 / 180) * PI
double g_max_turn = ((double)10.0 / 180) * M_PI; // rad/second
// should we continuously send commands?
bool g_always_command = false;

int g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;
bool g_done;



class TBK_Node
{

private:
  geometry_msgs::Twist cmdvel_;
  ros::NodeHandle n_;
  ros::Publisher pub_;

public:
  TBK_Node()
  {
    pub_ = n_.advertise<geometry_msgs::Twist> ("topic_teleop_base_teclado_pub", 1);
  }

  ~TBK_Node()
  {

  }

  void keyboardLoop()
  {
    char c;
    double max_tv = g_max_speed;
    double max_rv = g_max_turn;
    bool dirty = false;

    int speed = 0;
    int turn = 0;

    int speed_ant = 0;
    int turn_ant = 0;

    // get the console in raw mode
    tcgetattr(g_kfd, &g_cooked);
    memcpy(&g_raw, &g_cooked, sizeof(struct termios));
    g_raw.c_lflag &= ~(ICANON | ECHO);
    g_raw.c_cc[VEOL] = 1;
    g_raw.c_cc[VEOF] = 2;
    tcsetattr(g_kfd, TCSANOW, &g_raw);

    puts("Lectura de teclado");
    puts("---------------------------");
    puts("q/z : aumentar/disminuir vel.angular max y vel.lineal max en un 10%");
    puts("w/x : aumentar/disminuir vel.lineal  max en un 10%");
    puts("e/c : aumentar/disminuir vel.angular max en un 10%");
    puts("---------------------------");
    puts("Para mover el robot presionar:");
    puts("   u    i    o");
    puts("   j    k    l");
    puts("   m    ,    .");
    puts("cualquir otra tecla: Parar");
    puts("---------------------------");

    struct pollfd ufd;
    ufd.fd = g_kfd;
    ufd.events = POLLIN;
    for (;;)
    {
      boost::this_thread::interruption_point();
      // get the next event from the keyboard
      int num;
      if ((num = poll(&ufd, 1, 250)) < 0)
      {
        perror("poll():");
        return;
      }
      else if (num > 0)
      {
        if (read(g_kfd, &c, 1) < 0)
        {
          perror("read():");
          return;
        }
      }
      else
      {
        c = '0';
        //continue;
      }

      switch (c)
      {
        // Movimiento de traslacion en linea recta de frente.
        case KEYCODE_I:
          speed = 1;
          turn = 0;
          dirty = true;
          break;
          // Parar el movimiento.
        case KEYCODE_K:
          speed = 0;
          turn = 0;
          dirty = true;
          break;
          // Movimiento de traslacion circular horario de frente.
        case KEYCODE_O:
          speed = 1;
          turn = -1;
          dirty = true;
          break;
          // Movimiento de rotacion antihorario.
        case KEYCODE_J:
          speed = 0;
          turn = 1;
          dirty = true;
          break;
          // Movimiento de rotacion horario.
        case KEYCODE_L:
          speed = 0;
          turn = -1;
          dirty = true;
          break;
          // Movimiento de traslacion circular antihorario de frente.
        case KEYCODE_U:
          turn = 1;
          speed = 1;
          dirty = true;
          break;
          // Movimiento de traslación en linea recta marcha atras.
        case KEYCODE_COMMA:
          turn = 0;
          speed = -1;
          dirty = true;
          break;
          // KEYCODE_PERIOD --> tecla punto.
          // Movimiento de traslacion circular antihorario marcha atras.
        case KEYCODE_PERIOD:
          turn = 1;
          speed = -1;
          dirty = true;
          break;
          // Movimiento de traslacion circular horario marcha atras.
        case KEYCODE_M:
          turn = -1;
          speed = -1;
          dirty = true;
          break;
          // Incrementar el valor de la velocidad angular y de desplazamiento en un 10%.
        case KEYCODE_Q:
          max_tv += max_tv / 10.0;
          max_rv += max_rv / 10.0;

          //if (g_always_command){
          //   dirty = true;
          //}
          break;

          // Decrementar el valor de la velocidad angular y de desplazamiento en un 10%.
        case KEYCODE_Z:
          max_tv -= max_tv / 10.0;
          max_rv -= max_rv / 10.0;
          //if (g_always_command){
          //   dirty = true;
          //}
          break;
          // Incrementar el valor de la velocidad de desplazamiento en un 10%.
        case KEYCODE_W:
          max_tv += max_tv / 10.0;
          //if (g_always_command){
          //   dirty = true;
          //}
          break;

          // Decrementar el valor de la velocidad de desplazamiento en un 10%.
        case KEYCODE_X:
          max_tv -= max_tv / 10.0;
          //if (g_always_command){
          //   dirty = true;
          //}
          break;

          // Incrementar el valor de la velocidad angular en un 10%.
        case KEYCODE_E:
          max_rv += max_rv / 10.0;
          //if (g_always_command){
          //   dirty = true;
          //}
          break;

          // Decrementar el valor de la velocidad de desplazamiento en un 10%.
        case KEYCODE_C:
          max_rv -= max_rv / 10.0;
          //if (g_always_command){
          //   dirty = true;
          //}
          break;
          // Por defecto parar.
        default:
          speed = 0;
          turn = 0;
          dirty = true;
      } // fin switch

      if (dirty == true && (speed != speed_ant || turn != turn_ant))
      {
        printf("%c\n", c);
        speed_ant = speed;
        turn_ant = turn;
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        printf("\n");
        printf("----- ----- ----- ----- -----\n");
        printf("vel.linear: %g\n", cmdvel_.linear.x);
        printf("vel.angular:%g\n", cmdvel_.angular.z);
        printf("===== ===== ===== ===== =====\n");
        pub_.publish(cmdvel_);

      }
    } // fin for
  }

  void stopRobot()
  {
    cmdvel_.linear.x = cmdvel_.angular.z = 0.0;
    pub_.publish(cmdvel_);
  }
};

//TBK_Node* tbk;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "nodoTeleopBaseTecladoPub",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  TBK_Node tbk;

  //boost::thread t = boost::thread::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));
  boost::thread t = boost::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));

  ros::spin();
  t.interrupt();
  t.join();
  tbk.stopRobot();
  tcsetattr(g_kfd, TCSANOW, &g_cooked);
  return (0);
}
