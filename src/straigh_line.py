#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# Funcao que ira receber parametros do usuario e mover a tartaruga
def moveTartaruga():

    # Inicia um novo no
    rospy.init_node('simples_movimento', anonymous = True)
    # Cria objeto publisher
    velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
    # Instancia objeto do tipo Twist
    vel_msg = Twist()

    print("Vamos mover a tartaruga!")
    velocidade = input("Digite a velocidade desejada: ") 
    distancia = input("Digite a distancia desejada: ")
    paraFrente = input("Para frente? -->(1)SIM - (2)NAO <-- Resposta: ")

    # Verifica se o movimento e para frente ou para tras
    if(paraFrente):
        vel_msg.linear.x = abs(velocidade)
    else:
        vel_msg.linear.x = -abs(velocidade)

    # Como o movimento sera apenas no eixo x, entao:
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    distancia_atual = 0
    
    # Enquanto nenhum comando de parada for acionado
    while not rospy.is_shutdown():
        
        # Obtem o tempo atual
        t0 = rospy.Time.now().to_sec()

        # Move a tartaruga ate a distancia especificada
        while (abs(distancia_atual) < distancia):
            # Publica no topico cmd_vel a velocidade espeficada
            velocity_publisher.publish(vel_msg)
            # Calcula diferenca de tempo
            t1 = rospy.Time.now().to_sec()
            delta_t = t0 - t1 
            # Calcula distancia percorrida
            distancia_atual = velocidade*delta_t
            print "Distancia: %f" % distancia_atual

        # Se distancia desejada alcancada, pare a tartaruga
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        moveTartaruga()
    except rospy.ROSInterruptException:
        pass

