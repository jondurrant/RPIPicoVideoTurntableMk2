add_executable(${NAME}
        main.cpp
        BlinkAgent.cpp
        Agent.cpp
        SwitchMgr.cpp
        SwitchObserver.cpp
        GPIOInputMgr.cpp
        GPIOObserver.cpp
        Stepper28BYJ.cpp
        StepperObserver.cpp
        OledDisplay.cpp
	    RotEncAgent.cpp
		RotEncListener.cpp
		UI.cpp
		
		freertos_allocators.c
        uRosBridge.cpp
        uRosEntities.cpp
        pico_usb_transport.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     FreeRTOS-Kernel-Heap4
     PicoLed
     pico_ssd1306
     micro_ros
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     
target_compile_definitions(${NAME} PRIVATE
    configNUM_CORES=1
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)