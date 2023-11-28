#define NUM_MODULES 4
#define FINISH_COUNTER 10 // final speed
#define BUS_SIZE 5

//MESSAGES
#define ENGINE_MSG 15
#define BIUS_ZERO_MSG 0
#define BIUS_MSG 1
//COUNTER TO RESET
#define ZERO_MEASEGES_TO_RESET 4

// mtype = { IN_CIRLE_ORBIT, GO_TO_ELLIPTICAL_ORBIT, IN_ELLIPTICAL_ORBIT }; // положение станции
// mtype = { IN_CIRLE_ORBIT, GO_TO_ELLIPTICAL_ORBIT, GOING_TO_ELLIPTICAL_ORBIT, IN_ELLIPTICAL_ORBIT}; // положение станции

// mtype = { IN_CIRLE_ORBIT, GOING_TO_ELLIPTICAL_ORBIT, IN_ELLIPTICAL_ORBIT}; // положение станции
mtype = {NON_LANDING, START_LANDING, LANDING, LANDED}

mtype = { WORK, NOT_WORK}
mtype = { BKU, BIUS, ENGINE, MODULE }; // доступные модули

mtype = { NONE, ENABLE_ACCELEROMETERS, DISABLE_ACCELEROMETERS, START_ENGINE, STOP_ENGINE, RESET }; // команды
mtype = { SYNC }

mtype STATION_STATUS = NON_LANDING


mtype BIUS_STATE = NOT_WORK
mtype ENGINE_STATE = NOT_WORK

// 0 - для двигателя, 1 - для биуса.
chan command_bus[2] = [BUS_SIZE] of { mtype }; // ОТ БКУ К ДВИГАТЕЛЮ(0) И БИУСУ(1)
chan data_bus = [BUS_SIZE] of { byte } // ОТ ДВИГАТЕЛЮ(0) И БИУСА(1) К БКУ


chan turn = [1] of { mtype };

chan sync = [1] of {mtype}
chan addSync = [1] of {mtype}

int angleSpeed = 0;
int cur_zero_msg_counter = 0;


active proctype Enviroment() {
    do
    ::  turn ! BKU
        // sync ? BKU

        turn ! BIUS
        // sync ? BIUS

        turn ! ENGINE
        // sync ? ENGINE
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
                ::  STATION_STATUS == START_LANDING -> 
                        command_bus[1] ! ENABLE_ACCELEROMETERS;
                        command_bus[0] ! START_ENGINE;
                        STATION_STATUS = LANDING;
                        // sync ! BKU
                :: cur_angle_speed == FINISH_COUNTER -> 
                        command_bus[1] ! DISABLE_ACCELEROMETERS
                        command_bus[0] ! STOP_ENGINE
                        // sync ! BKU

                :: nempty(data_bus) ->
                    data_bus ? var
                    if 
                        :: var == BIUS_ZERO_MSG ->
                            cur_zero_msg_counter = cur_zero_msg_counter + 1
                        :: var == ENGINE_MSG ->
                            var = 0
                        :: else -> skip
                    fi

                    if 
                        ::  cur_zero_msg_counter >= ZERO_MEASEGES_TO_RESET ->
                                BIUS_STATE = RESET
                                cur_zero_msg_counter = 0
                        :: else -> skip
                    fi

                    cur_angle_speed = cur_angle_speed + var;
                    if 
                        :: cur_angle_speed == FINISH_COUNTER ->
                            ENGINE_STATE = STOP_ENGINE
                            BIUS_STATE = DISABLE_ACCELEROMETERS
                            STATION_STATUS = LANDED
                            cur_angle_speed = 0;
                        :: else -> skip
                    fi
                :: empty(data_bus) -> skip // poka tak
                // :: cur_zero_msg_counter >= ZERO_MEASEGES_TO_RESET ->
                //     BIUS_STATE = RESET
                //     cur_zero_msg_counter = 0

                        // sync ! BKU
                // :: BIUS_STATE == WORK && full(data_bus) -> 
                //         BIUS_STATE = RESET;
                //         // sync ! BKU
                // :: BIUS_STATE == RESET && data_bus == BUS_SIZE -> skip
                :: STATION_STATUS == LANDED -> 
                    // sync ! BKU
                    skip
            fi;
    od;
}

// ЕСЛИ ТУТ НЕ ПОДОШЛО ХОТЬ ОДНО ОН БЛОЧИТСЯ И ОТДАЕМ УПРАВЛЕНИЕ, ПОТОМ ЕСЛИ ХОТЬ ОДНО УСЛОВИЕ TRUE, ПРОСЫПАЕТСЯ

// БИУС-Л
/* 
    В начале: акселерометры отключены, показания с датчиков не снимаются. YES
    В отключенном состоянии БИУС-Л подает штатные нулевые данные, называемые «нулевым сигналом». YES

    Когда получает команду «Включить акселерометры», БИУС-Л запускает акселерометры, 
    начинает снимать показания с датчиков. БИУС-Л передает данные об угловой скорости БКУ. YES

*/
//id = 1
active proctype Bius() {
    do
    ::  turn ? BIUS ->
            if 
                :: command_bus[1] ? ENABLE_ACCELEROMETERS ->
                    BIUS_STATE = WORK
                    data_bus ! BIUS_MSG
                    // sync ! BIUS
                :: command_bus[1] ? DISABLE_ACCELEROMETERS ->
                    BIUS_STATE = NOT_WORK
                        // sync ! BIUS
                :: BIUS_STATE == NOT_WORK ->
                    addSync ! SYNC // тут мб, что другой поставит синк, а этот зайдет, но вроде это безболезненно  - не. дичь написал
                    if 
                        :: len(data_bus) != BUS_SIZE -> data_bus ! BIUS_ZERO_MSG;
                        :: len(data_bus) == BUS_SIZE -> skip
                    fi
                    addSync ? SYNC
                        // data_bus ! BIUS_ZERO_MSG;
                        // sync ! BIUS
                :: BIUS_STATE == WORK /*&& data_bus != BUS_SIZE*/ ->
                    addSync ! SYNC
                    if 
                        :: len(data_bus) != BUS_SIZE -> data_bus ! BIUS_MSG
                        :: len(data_bus) == BUS_SIZE -> skip
                    fi
                    addSync ? SYNC
                // :: BIUS_STATE == WORK && data_bus == BUS_SIZE -> skip // на счет этого пока не знаю
                        // sync ! BIUS
                // :: BIUS_STATE == WORK && full(data_bus) -> skip // на счет этого пока не знаю
                // :: command_bus[1] ? RESET -> 
                :: BIUS_STATE == RESET -> 
                    do
                        :: nempty(command_bus[1]) ->
                            command_bus[1] ? _;
                        :: empty(command_bus[1]) ->
                            break;
                    od;

                    BIUS_STATE = NOT_WORK

                    // sync ! BIUS
                :: STATION_STATUS == LANDED -> 
                    // sync ! BIUS
                    skip
            fi
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
                    data_bus ! ENGINE_MSG 
                        // sync ! ENGINE
                :: command_bus[0] ? STOP_ENGINE ->
                    ENGINE_STATE = NOT_WORK
                        // sync ! ENGINE
                :: ENGINE_STATE == WORK /*&& data_bus != BUS_SIZE*/ ->
                    addSync ! SYNC
                    if 
                        :: len(data_bus) != BUS_SIZE -> data_bus ! ENGINE_MSG
                        :: len(data_bus) == BUS_SIZE -> skip
                    fi
                    addSync ? SYNC
                :: ENGINE_STATE == NOT_WORK -> skip
                        // data_bus ! ENGINE_MSG // тут таймаут, когда переполнено и больше не может класть
                        // sync ! ENGINE
                // :: ENGINE_STATE == WORK && data_bus == BUS_SIZE -> skip // на счет этого пока не знаю
                // :: else -> skip
                :: STATION_STATUS == LANDED -> 
                    // sync ! ENGINE
                    skip
            fi
    od
}

active proctype start() {
    STATION_STATUS = START_LANDING
}


// ltl {<> (STATION_STATUS == RESET)}
// ltl {<> (len(data_bus) == BUS_SIZE)}
// ltl {<> (cur_zero_msg_counter == ZERO_MEASEGES_TO_RESET)}
// ltl {<>[] (STATION_STATUS == LANDED)}
// ltl {[] (STATION_STATUS == NON_LANDING)}
ltl {[] (BIUS_STATE == RESET -> <> ((BIUS_STATE == WORK) U (STATION_STATUS == LANDED)))}



// ltl {(BIUS_STATE == RESET && len(data_bus[1]) == BUS_SIZE) -> (BIUS_STATE == WORK)}
// ltl {(BIUS_STATE == NOT_WORK && len(data_bus) == BUS_SIZE) -> ([]<> BIUS_STATE == WORK)}

// Столкновение происходит, когда передается слишком много "нулевых сигналов", БКУ дает команду "Перезагрузить", при перезагрузке
// очищает массив входящих сигналов и пропускает команду "Включить акслерометры". (Т.е. он подавал много нулевых до того, как включить акселерометры было подано, в итоге команда включить попала в канал, но и так же была конмада резета, он сделал резет и пропустил команду включить акселеромтеры) -> фикс -  при сбросе бежать по массиву и выбирать по одной,
// если встретилась включить акслелерометры, то включаем их и дальше очищаем 


//КУ может передавать команды в шину команд, остальные модули только получать их. БИУС-Л, 
//двигатель, другие модули могут передавать данные в шину передачи данных, БКУ только получает их.



//55:36
