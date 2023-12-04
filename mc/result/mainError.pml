#define NUM_MODULES 4
#define FINISH_ANGLE_SPEED 10 // final speed
#define BUS_SIZE 5

//MESSAGES
#define ENGINE_MSG 15
#define BIUS_ZERO_MSG -1
#define BIUS_MSG 1
#define MODULE_MSG 14
#define ZERO_MEASEGES_TO_RESET 4 //COUNTER TO RESET

mtype = { IN_CIRLE_ORBIT, GO_TO_ELLIPTICAL_ORBIT, GOING_TO_ELLIPTICAL_ORBIT, IN_ELLIPTICAL_ORBIT}; // положение станции
mtype = { WORK, NOT_WORK}
mtype = { BKU, BIUS, ENGINE, MODULES }; // доступные модули

mtype = { NONE, ENABLE_ACCELEROMETERS, DISABLE_ACCELEROMETERS, START_ENGINE, STOP_ENGINE, RESET }; // команды
mtype = { SYNC }

mtype STATION_STATUS = IN_CIRLE_ORBIT
mtype BIUS_STATE = NOT_WORK
mtype ENGINE_STATE = NOT_WORK
mtype MODULES_STATE = NOT_WORK

// 0 - для двигателя, 1 - для биуса.
chan command_bus[2] = [BUS_SIZE] of { mtype }; // ОТ БКУ К ДВИГАТЕЛЮ(0) И БИУСУ(1)
chan data_bus[3] = [1] of { int } // ОТ ДВИГАТЕЛЮ(0) И БИУСА(1) К БКУ (по идее если биус отсылает скорость аппарата угловую, то надо, чтобы размер был 1, чтобы не копились скорости, а если есть в канале, то текущую считало бы)
chan stop_reported = [2] of { int }

chan turn = [1] of { mtype };
chan sync = [1] of {mtype}

int cur_zero_msg_counter = 0;

bool enableAccelerometersSended = false

bool biusEnabledSend = false
bool resetSend = false

bool isBiusEnabled = false
bool isEngineEnabled = false

active proctype Enviroment() {
    do
    ::  turn ! BKU
        sync ? BKU

        turn ! BIUS
        sync ? BIUS

        turn ! ENGINE
        sync ? ENGINE

        // turn ! MODULES
        // sync ? MODULES
    od
}


// Бортовой комплекс управления:
/*
 БКУ может передавать команды в шину команд, остальные модули только получать их. БИУС-Л, 
 двигатель, другие модули могут передавать данные в шину передачи данных, БКУ только получает их. 
 Среда коммуникации опрашивает компоненты по алгоритму round-robin, то есть опрашивает каждую компоненту по кругу. 
 На каждом таком круге модуль либо получает информацию, либо отправляет ее.
*/
/*
    1) При посадке: выдает команды о переходе на эллиптическую орбиту: БКУ выдает БИУС-Л команду 
    «Включить акселерометры», БКУ выдает двигателю команду «Запустить двигатель». YES
    2) Фиксирует вывод аппарата на эллиптическую орбиту, Когда получает от БИУС-Л и двигателя инфу. После этого БКУ 
    должен отключить двигатель, подав команду «Остановить двигатель», также БКУ отключает БИУС-Л, 
    подав команды «Отключить акселерометры». 
    3) В случае необходимости БКУ может перезагрузить БИУС-Л, подав команду «Перезагрузить». 
    При перезагрузке БИУС-Л очищает все пришедшие к нему команды и начинает работу с выключенного состояния. YES

*/

active proctype Bku() {
    int cur_angle_speed = 0;
    int var = 0;
    do 
    ::  turn ? BKU ->
            if 
                ::  STATION_STATUS == GO_TO_ELLIPTICAL_ORBIT -> 
                        command_bus[1] ! ENABLE_ACCELEROMETERS;
                        command_bus[0] ! START_ENGINE;
                        biusEnabledSend = true
                        enableAccelerometersSended = true
                        STATION_STATUS = GOING_TO_ELLIPTICAL_ORBIT;
                        cur_zero_msg_counter = 0 // добавил, мб неправильно

                        //clear from zero data
                        do
                        :: nempty(data_bus[1]) ->
                            data_bus[1] ? _;
                        :: empty(data_bus[1]) ->
                            break;
                        od;
                        
                        goto end
                :: STATION_STATUS == IN_ELLIPTICAL_ORBIT -> skip
                :: STATION_STATUS == IN_CIRLE_ORBIT -> STATION_STATUS = GO_TO_ELLIPTICAL_ORBIT
                :: STATION_STATUS == IN_CIRLE_ORBIT || STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT -> skip
            fi;

            if
                :: full(stop_reported) -> STATION_STATUS == IN_ELLIPTICAL_ORBIT
                :: nfull(stop_reported) -> skip
            fi

            //if for in_circle orbit + going_to_elliptical
            if //bius if
                :: nempty(data_bus[1]) ->
                    data_bus[1] ? var -> 
                    if 
                        :: var == BIUS_ZERO_MSG ->
                            cur_zero_msg_counter = cur_zero_msg_counter + 1
                        :: else -> cur_zero_msg_counter = 0
                    fi

                    if 
                        ::  cur_zero_msg_counter >= ZERO_MEASEGES_TO_RESET ->
                                command_bus[1] ! RESET
                                resetSend = true
                                cur_zero_msg_counter = 0
                        :: else -> skip
                    fi

                    if 
                        :: var != BIUS_ZERO_MSG && var == FINISH_ANGLE_SPEED ->
                            command_bus[0] ! STOP_ENGINE
                            command_bus[1] ! DISABLE_ACCELEROMETERS
                            STATION_STATUS = IN_ELLIPTICAL_ORBIT // mb менять, когда stop reported

                            cur_angle_speed = 0;
                            cur_zero_msg_counter = 0
                        :: else -> skip
                    fi
                :: empty(data_bus[1]) -> skip
            fi

            if //engine if
                :: nempty(data_bus[0]) ->
                    data_bus[0] ? var -> 
                :: empty(data_bus[0]) -> skip
            fi

            // if //modules
            //     :: nempty(data_bus[2]) ->
            //         data_bus[2] ? var -> 
            //     :: empty(data_bus[2]) -> skip
            // fi

            end: skip
            sync ! BKU
    od;
}

// БИУС-Л
/* 
    В начале: акселерометры отключены, показания с датчиков не снимаются. YES
    В отключенном состоянии БИУС-Л подает штатные нулевые данные, называемые «нулевым сигналом». YES

    Когда получает команду «Включить акселерометры», БИУС-Л запускает акселерометры, 
    начинает снимать показания с датчиков. БИУС-Л передает данные об угловой скорости БКУ. YES

*/
//id = 1
active proctype Bius() {
    int angleSpeed = 0;
    do
    ::  turn ? BIUS ->

            // if для чтения
            if 
                :: command_bus[1] ? ENABLE_ACCELEROMETERS ->
                    BIUS_STATE = WORK
                    isBiusEnabled = true
                    biusEnabledSend = false
                    goto biusEnd
                    
                :: command_bus[1] ? DISABLE_ACCELEROMETERS ->
                    BIUS_STATE = NOT_WORK
                    stop_reported ! 1
                    isBiusEnabled = false
                    goto biusEnd

                :: command_bus[1] ? RESET ->
                    do
                        :: nempty(command_bus[1]) ->
                            command_bus[1] ? _;
                        :: empty(command_bus[1]) ->
                            break;
                    od;
                    isBiusEnabled = false
                    BIUS_STATE = NOT_WORK
                    goto biusEnd

                :: empty(command_bus[1]) -> skip
            // fi


            // if для записи 
                :: BIUS_STATE == NOT_WORK ->
                    if 
                        :: nfull(data_bus[1]) ->
                            data_bus[1] ! BIUS_ZERO_MSG;
                        :: full(data_bus[1]) -> skip
                    fi
                :: BIUS_STATE == WORK -> 
                    if
                        :: nfull(data_bus[1]) ->
                            data_bus[1] ! angleSpeed

                            if
                                :: angleSpeed + 1 <= FINISH_ANGLE_SPEED ->
                                    angleSpeed++
                                :: else -> skip
                            fi
                        :: full(data_bus[1]) -> skip
                    fi
                :: full(stop_reported) && STATION_STATUS == IN_ELLIPTICAL_ORBIT -> skip
            fi

            biusEnd: skip
            sync ! BIUS
    od;
}


// Двигатель
/* 
    В начале: выключен YES
    Получив команду «Запустить двигатель», двигатель запускается. Двигатель начинает передавать данные о скорости БКУ. YES
*/

// id = 0
active proctype Engine() {
    do 
    :: turn ? ENGINE ->
            // if для чтения
            if
                ::  command_bus[0] ? START_ENGINE ->
                    ENGINE_STATE = WORK
                    goto engineEnd

                :: command_bus[0] ? STOP_ENGINE ->
                    ENGINE_STATE = NOT_WORK
                    stop_reported ! 1
                    goto engineEnd

                :: empty(command_bus[0]) -> skip
            fi

            // if для записи
            if
                :: ENGINE_STATE == WORK ->
                    if 
                        :: nfull(data_bus[0]) ->
                            data_bus[0] ! ENGINE_MSG
                        :: full(data_bus[0]) -> skip
                    fi
                :: ENGINE_STATE == NOT_WORK -> skip
                :: full(stop_reported) && STATION_STATUS == IN_ELLIPTICAL_ORBIT -> skip
            fi

            engineEnd: skip
            sync ! ENGINE
    od
}

// id = 2
// Всегда что-то отправляют, но они никак не влиют, по заданию нужны, пусть будут, хотя их лучше убрать, т.е только больше состояний делают
// active proctype Modules() {
//     do 
//     :: turn ? MODULES ->
//             if
//                 :: MODULES_STATE == WORK ->
//                     data_bus[2] ! MODULE_MSG
//                 :: MODULES_STATE == NOT_WORK -> skip
//             fi
//             sync ! MODULES
//     od
// }

// active proctype start() {
//     STATION_STATUS = GO_TO_ELLIPTICAL_ORBIT
// }


// ltl {<> (STATION_STATUS == RESET)}
// ltl {<> (len(data_bus) == BUS_SIZE)}
// ltl {<> (cur_zero_msg_counter == ZERO_MEASEGES_TO_RESET)}
// ltl {<>[] (STATION_STATUS == LANDED)}
// ltl {[] (STATION_STATUS == NON_LANDING)}
// ltl {[] ((BIUS_STATE == RESET) -> <>[] ((BIUS_STATE == WORK) U (STATION_STATUS != LANDED)))}
// ltl {<> (STATION_STATUS == IN_ELLIPTICAL_ORBIT)}
// ltl {(STATION_STATUS == GO_TO_ELLIPTICAL_ORBIT) -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT)}


// ltl {[] ((enableAccelerometersSended == true) -> <>((BIUS_STATE == WORK) U (STATION_STATUS == IN_ELLIPTICAL_ORBIT)))}


// ltl {[] (resetSend && enableAccelerometersSended -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT))}
// ltl {[] (resetSend && enableAccelerometersSended -> <> (BIUS_STATE == WORK U (BIUS_STATE !=NOT_WORK && STATION_STATUS == IN_ELLIPTICAL_ORBIT)))} //dich mb
// ltl {[] (resetSend && enableAccelerometersSended -> <> (BIUS_STATE == WORK U STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT))}
// ltl {[](biusEnabledSend -> <> isBiusEnabled)}


// ltl {[]((STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT) -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT))}
ltl {[]((biusEnabledSend) -> <> (isBiusEnabled))}


// ltl {(BIUS_STATE == RESET && len(data_bus[1]) == BUS_SIZE) -> (BIUS_STATE == WORK)}
// ltl {(BIUS_STATE == NOT_WORK && len(data_bus) == BUS_SIZE) -> ([]<> BIUS_STATE == WORK)}

// Столкновение происходит, когда передается слишком много "нулевых сигналов", БКУ дает команду "Перезагрузить", при перезагрузке
// очищает массив входящих сигналов и пропускает команду "Включить акслерометры". (Т.е. он подавал много нулевых до того, как включить акселерометры было подано, в итоге команда включить попала в канал, но и так же была конмада резета, он сделал резет и пропустил команду включить акселеромтеры) -> фикс -  при сбросе бежать по массиву и выбирать по одной,
// если встретилась включить акслелерометры, то включаем их и дальше очищаем 


//КУ может передавать команды в шину команд, остальные модули только получать их. БИУС-Л, 
//двигатель, другие модули могут передавать данные в шину передачи данных, БКУ только получает их.



//55:36
// с sync все равно такая же дичь, крч проблема в том, что если бку читает по одной, то в любом случае когда-то таймаут будет, если блочится на вставлении в канал, можно обходить проверку на полному и скипать, но правильно ли это, если сказано, что не должны теряться сообщения и дб приняты сообщения или посланы на каждом шаге