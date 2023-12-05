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
chan roundSync = [3] of {mtype}

int cur_zero_msg_counter = 0;

bool enableAccelerometersSended = false

bool biusEnabledSend = false
bool biusDisabledSend = false
bool engineDisabledSend = false
bool resetSend = false

bool isBiusEnabled = false
bool isEngineEnabled = false

active proctype Enviroment() {
    do
    ::  turn ! BKU
        // sync ? BKU

        turn ! BIUS
        // sync ? BIUS

        turn ! ENGINE
        // sync ? ENGINE

        // turn ! MODULES
        // sync ? MODULES

        roundSync ? SYNC
        roundSync ? SYNC
        roundSync ? SYNC
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
    int var = 0;
    do 
    ::  turn ? BKU ->
            if 
                ::  STATION_STATUS == GO_TO_ELLIPTICAL_ORBIT -> 
                        biusEnabledSend = true
                        enableAccelerometersSended = true
                        STATION_STATUS = GOING_TO_ELLIPTICAL_ORBIT;
                        cur_zero_msg_counter = 0 // добавил, мб неправильно

                        command_bus[1] ! ENABLE_ACCELEROMETERS;
                        command_bus[0] ! START_ENGINE;

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
                        :: !biusDisabledSend && !engineDisabledSend && var != BIUS_ZERO_MSG && var == FINISH_ANGLE_SPEED ->
                            command_bus[0] ! STOP_ENGINE
                            command_bus[1] ! DISABLE_ACCELEROMETERS
                            biusDisabledSend = true
                            engineDisabledSend = true
                            STATION_STATUS = IN_ELLIPTICAL_ORBIT
                            biusDisabledSend = true

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

            roundSync ! SYNC
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
    mtype command = NONE
    do
    ::  turn ? BIUS ->
            if 
                :: command_bus[1] ? ENABLE_ACCELEROMETERS ->
                    acc: skip

                    biusEnabledSend = false
                    BIUS_STATE = WORK
                    isBiusEnabled = true
                    goto biusEnd
                    
                :: command_bus[1] ? DISABLE_ACCELEROMETERS ->
                    BIUS_STATE = NOT_WORK
                    stop_reported ! 1
                    isBiusEnabled = false
                    goto biusEnd

                :: command_bus[1] ? RESET ->
                    bool containsEnable = false
                    do
                        :: nempty(command_bus[1]) ->
                            command_bus[1] ? command;
                            if
                                :: command == ENABLE_ACCELEROMETERS -> containsEnable = true
                                :: else -> skip
                            fi
                        :: empty(command_bus[1]) ->
                            break;
                    od;
                    isBiusEnabled = false
                    BIUS_STATE = NOT_WORK

                    if
                        :: containsEnable -> goto acc
                        :: else -> skip
                    fi

                    goto biusEnd
                    
                :: empty(command_bus[1]) -> skip
            fi

            if
                :: BIUS_STATE == NOT_WORK ->
                            data_bus[1] ! BIUS_ZERO_MSG;
                :: BIUS_STATE == WORK -> 
                            data_bus[1] ! angleSpeed

                            if
                                :: angleSpeed + 1 <= FINISH_ANGLE_SPEED ->
                                    angleSpeed++
                                :: else -> skip
                            fi
                :: full(stop_reported) && STATION_STATUS == IN_ELLIPTICAL_ORBIT -> skip
            fi

            biusEnd: skip
            roundSync ! SYNC
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

            if
                :: ENGINE_STATE == WORK ->
                    data_bus[0] ! ENGINE_MSG

                :: ENGINE_STATE == NOT_WORK -> skip

                :: full(stop_reported) && STATION_STATUS == IN_ELLIPTICAL_ORBIT -> skip
            fi

            engineEnd: skip
            roundSync ! SYNC
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


// ltl {[]((STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT) -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT))}
// ltl {[]((biusEnabledSend) -> <> (isBiusEnabled))}
// ltl {<> (STATION_STATUS == IN_ELLIPTICAL_ORBIT)}
// ltl {(STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT) -> <>(STATION_STATUS == IN_ELLIPTICAL_ORBIT) }


//finals
ltl {[]((biusEnabledSend) -> <> (BIUS_STATE == WORK))}
ltl {[]((STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT) -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT))}
// эти сверъху находят проблему, когда резет очищает ACCELEROMETRS_ENABLE

ltl {[]((STATION_STATUS == GOING_TO_ELLIPTICAL_ORBIT && BIUS_STATE == WORK && ENGINE_STATE == WORK) -> <> (STATION_STATUS == IN_ELLIPTICAL_ORBIT))} // ВСЕГДА если состоние, что начали идти на эллиптическую орбиту и двигатель с биусом запустились, то достигнем эллиптеческой орбиты


// Столкновение происходит, когда передается слишком много "нулевых сигналов", БКУ дает команду "Перезагрузить", при перезагрузке
// очищает массив входящих сигналов и пропускает команду "Включить акслерометры". (Т.е. он подавал много нулевых до того, как включить акселерометры было подано, в итоге команда включить попала в канал, но и так же была конмада резета, он сделал резет и пропустил команду включить акселеромтеры) -> фикс -  при сбросе бежать по массиву и выбирать по одной,
// если встретилась включить акслелерометры, то включаем их и дальше очищаем 
