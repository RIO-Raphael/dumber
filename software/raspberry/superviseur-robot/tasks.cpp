/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TCHECKBATTERY 20
#define CST_PERTE_MEDIUM_ROBOT 3

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write!
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck!
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;
    b_reloadWD=false;
    WD=false;
    robotStarted=0;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_WD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cam, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_shutCamera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_send_image, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_image, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Mutexes created successfully" << endl << flush;
    
    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_Reload, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_useCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_useCamera, "th_useCamera", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::reloadWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_start(&th_useCamera, (void(*)(void*)) & Tasks::UseCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;
    
    //##### QST ###########
    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_mutex_release(&mutex_monitor);
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "\n Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write    //A FAIRE Travail sur les 10 ms en etat de Recherche incomprise 
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            Stop(); //Fonction 6 : On stoppe le robot //On arrete le serveur
            //On est dans l'état de démarrage du superviseur
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
            rt_sem_broadcast(&sem_startRobot);
            rt_mutex_acquire(&mutex_WD, TM_INFINITE);
            WD=1;
            rt_mutex_release(&mutex_WD);                    
        }else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
            rt_mutex_acquire(&mutex_WD, TM_INFINITE);
            WD=0;
            rt_mutex_release(&mutex_WD);
        }else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            cout << "Avec supplément chocolat !" << endl;
            rt_sem_v(&sem_openCamera);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread closing communication with the robot.
 */
void Tasks::CloseComRobot(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1){
        rt_sem_p(&sem_closeComRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        if (robot.Close()<0){
            //Fail
            Message * msgSend=new Message(MESSAGE_ANSWER_ROBOT_ERROR);
            WriteInQueue(&q_messageToMon, msgSend);
            cout << "Le robot s'est mal fermé." << endl << flush;
        }else{
            //Succes
            Message * msgSend=new Message(MESSAGE_ANSWER_ACK);
            WriteInQueue(&q_messageToMon, msgSend);
            cout << "Le robot s'est bien fermé." << endl << flush;
        }
        rt_mutex_release(&mutex_robot);
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        
        perteComRobot=0;
        
        if (WD){
            //on démarre avec le WD
            msgSend = WriteToRobot(robot.StartWithWD());
            cout << "Start robot with watchdog (";
        }else{
            //on démarre sans le WD
            msgSend = WriteToRobot(robot.StartWithoutWD());
            cout << "Start robot without watchdog (";
        }
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "Movement answer: " << msgSend->ToString() << endl;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
        
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            rt_mutex_acquire(&mutex_WD, TM_INFINITE);
            if(WD==1){
               rt_sem_broadcast(&sem_Reload); 
            }          
            rt_mutex_release(&mutex_WD);
        }else{
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
          
            WriteToRobot(new Message((MessageID)cpMove));
        }
        cout << endl << flush;
    }
}

/**
* @brief Thread handling battery checking.
*/
void Tasks::CheckBattery(void *arg){
    int rs;
    MessageBattery* msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while(1){
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_task_wait_period(NULL);
            cout << "Periodic checking of battery level : " << endl << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, msg); // or monitor.Write(msg); 
            // msg is deleted after being sent
        }
    }
}

/**********************************************************************/
/* Queue services                                                     */
/**********************************************************************/

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

//On écrit un message et on envoie un ordre au robot
/**
 * Envoie un ordre au robot. Gestion de la perte de communication avec le robot
 * @param msg Message à envoyer au robot
 * @return Message de retour provenant du robot
 */
Message* Tasks::WriteToRobot(Message* orderRobot){
    Message* answerRobot;
    Message* msgRobotComClose;
   
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    answerRobot = robot.Write(orderRobot);
    rt_mutex_release(&mutex_robot);
    //Ici vérification que le message a bien été écrit
    //si c'est le cas : mettre à zéro la variable globale perte_ComRobot
    //si ce n'est pas le cas : incrémenter la variable perte_ComRobot
    /* Note : le codeur a estimé qu'une erreur de communication peut prendre
     * plusieurs formes. Notamment, la connexion est estimée perdue si :
     * - le timeout est dépassé,
     * - la commande n'est pas reconnue,
     * - la commande n'est pas conforme à sa définition
     * et ce, au moins trois fois d'affilée.
     */
    rt_mutex_acquire(&mutex_perteComRobot, TM_INFINITE);
    if (answerRobot->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)
        || answerRobot->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND)
        || answerRobot->CompareID(MESSAGE_ANSWER_ROBOT_ERROR)) {
        perteComRobot++;
        //cout << "ERROR ++" << endl << flush;
    }
    else {
        perteComRobot = 0;
        //cout << "ERROR = 0" << endl << flush;
    }
    //Gestion de perteComRobot
    if (perteComRobot > 3){
        cout << "COMROBOT LOST" << endl << flush;
        msgRobotComClose = new Message(MESSAGE_ANSWER_COM_ERROR);
        WriteInQueue(&q_messageToMon, msgRobotComClose); // msg is deleted after being sent
    }
    rt_mutex_release(&mutex_perteComRobot);
    return answerRobot;
}

void Tasks::reloadWD(void){
    bool ok_WD=false;
    int compt_erreur_ACK=0;
    int rs=0;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    if(rt_task_set_periodic(NULL,TM_NOW,1e9)!=0){cout<<"Problème Task ReloadWD()"<<endl;};
    
    //Etat d'attente
    while (1){
        rt_sem_p(&sem_Reload, TM_INFINITE);

        
        // La var WD est mise à 1 dans le receivefromMon        
        rt_mutex_acquire(&mutex_WD, TM_INFINITE);
        ok_WD=WD;
        rt_mutex_release(&mutex_WD);
        
        while(ok_WD){
            rt_task_wait_period(NULL);
            
            //On vérifie que le robot soit encore actif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            
            if (rs==1){
            
                //On envoie le message de reload
                Message* msg_reloadWD=new Message(MESSAGE_ROBOT_RELOAD_WD);
                msg_reloadWD=WriteToRobot(msg_reloadWD);
                
                //On regarde la réponse
                if (!(msg_reloadWD->CompareID(MESSAGE_ANSWER_ACK))){
                    compt_erreur_ACK++;
                }else{
                    compt_erreur_ACK--;
                    if (compt_erreur_ACK<=0){compt_erreur_ACK=0;}
                }
                cout << "Message Reload envoyé" << endl;
            }else{
                rt_mutex_acquire(&mutex_WD, TM_INFINITE);
                WD=0;
                cout << "On arrête le Watchdog." << endl;
                rt_mutex_release(&mutex_WD);
            }
            
            cout << "Compteur ACK : " << compt_erreur_ACK <<endl <<endl <<endl;
            
            if (compt_erreur_ACK>2){
                ok_WD=0;
                //Eteindre le robot
                cout << "On éteint le robot." << endl;
                rt_sem_broadcast(&sem_closeComRobot);
            }else{
                //MAJ ok_WD
                rt_mutex_acquire(&mutex_WD, TM_INFINITE);
                ok_WD=WD;
                rt_mutex_release(&mutex_WD);
            }
        }
    } 
}

void Tasks::OpenCamera(){
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while (1){
        //On attend l'ordre d'ouvrir la caméra
        rt_sem_p(&sem_openCamera, TM_INFINITE);

        Message* msgSend;
        //On va allumer la cam
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        if (cam.Open()){
            //La caméra est ouverte
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            WriteInQueue(&q_messageToMon, msgSend);

            //On initialise les var cam
            //Pas besoin du mutex car les autres taches ne tournent pas encore
            shutCamera=false;
            send_image=true;
            cout << "la caméra est allumée" << endl;
            
            //On lance l'acquisition des images
            rt_sem_broadcast(&sem_useCamera);
            cout << "On lance l'acquisition de l'image." << endl;
        }else{
            //La caméra n'est pas ouverte
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgSend);

            cout << "la caméra n'est pas allumée" << endl;
        }
        rt_mutex_release(&mutex_cam);
    }   
}

void Tasks::UseCamera(){
    bool shutCamAux=false;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    if(rt_task_set_periodic(NULL,TM_NOW,1e8)!=0){cout<<"Problème Task UseCamera()"<<endl;}; //100ms
    
    while (1){
        //On attend l'ordre d'ouvrir la caméra
        rt_sem_p(&sem_useCamera, TM_INFINITE);
        
        while (!shutCamAux){
            //ON attend la période
            rt_task_wait_period(NULL);
            
            //On regarde si on doit envoyer les images
            rt_mutex_acquire(&mutex_send_image, TM_INFINITE);
            if (send_image){
                //On aurait pu faire qu'un seul mutex
                rt_mutex_acquire(&mutex_image, TM_INFINITE);
                rt_mutex_acquire(&mutex_cam, TM_INFINITE);
                Img image=cam.Grab();
                cout << image.ToString();
                MessageImg* msgImg = new MessageImg(MESSAGE_CAM_IMAGE,&image);
                WriteInQueue(&q_messageToMon, msgImg);
                rt_mutex_release(&mutex_cam);
                rt_mutex_release(&mutex_image);
            }
            rt_mutex_release(&mutex_send_image);
            
            //MAJ shutCamAux
            rt_mutex_acquire(&mutex_shutCamera, TM_INFINITE);
            shutCamAux=shutCamera;
            rt_mutex_release(&mutex_shutCamera);
            
        }
        
        //On regarde si on eteint la cam entre temps
       
        
    }   
}
