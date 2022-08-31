Declare Sub Get_speedknob_value()
Declare Sub Control_motor_speed() ```
Declare Sub Stopmotorandwaitforbuttons()
Declare Sub Read_buttons()
Declare Sub Show_status()
Declare Sub Switch_active_motor()
Declare Sub My_getadc(byval Channel As Byte)

$crystal = 4000000
Config Timer0 = Timer , Prescale = 8
Config Timer1 = Pwm , Pwm = 8 , Compare_a_pwm = Clear_down , Prescale = 1
Config Adc = Single , Prescaler = 64

Portb = &HFF
Portc = &B11111100
Portd = &HFF

'Config Pinb.0 = Output                                      'Open=Motor1 Enable; 1=Disable
'Config Pinb.1 = Output                                      'PWM (0=motor on)
'Config Pinb.2 = Output                                      'Open=Motor2 Enable; 1=Disable
Ddrb = &B11111111
'Used    .....XXX

'Config Pinc.0 = Input                                       'ADC0 Motor 1 speed
'Config Pinc.1 = Input                                       'ADC1 Motor 2 speed
'Config Pinc.2 = Input                                       'ADC2 0=Photo2 light
'Config Pinc.3 = Input                                       'ADC3 0=Photo1 light
'Config Pinc.4 = Input                                       'ADC4 SpeedKnob
Ddrc = &B11100000
'Used    ...XXXXX

'Config Pind.0 = Output                                      '0=LED1 on
'Config Pind.1 = Output                                      '0=LED2 on
'Config Pind.2 = Output                                      '0=LED3 on (mode)
'Config Pind.6 = Input                                       ' 0=Button1 pressed with PullUp enabled
'Config Pind.7 = Input                                       ' 0=Button2 pressed with PullUp enabled
Ddrd = &B00111111
'Used    XX...XXX

'
'
'
Dim Motorpwm As Byte
Dim Speedknobvalue As Word
Dim Motorspeed As Word
Dim Activemotor As Byte
Dim Targetspeed As Word
Dim Tmr1 As Word
Dim Tmr2 As Word
Dim Operationmode As Byte
Dim State As Byte
Dim Buttons As Byte
Dim Delayup As Byte
Dim Delaydown As Byte
Dim Rounds As Byte
Dim Channel As Byte
Dim Stopping As Byte

'
' Work variables
'
Dim W As Word
'Dim B As Word
'Dim B1 As Word
'Dim B2 As Word

'
' Initialisation
'
Pwm1a = 0

On Timer0 Timer0_isr
Start Timer0
Enable Timer0
Enable Interrupts

'Targetspeed = 0
'Motorspeed = 0
'Motorpwm = 0
Activemotor = 1
Operationmode = 0
State = 0
Stopping = 0

'
' Disable motors
'

Start Adc                                                   ' Powerup ADC
Stopmotorandwaitforbuttons

Do
   Control_motor_speed
   Get_speedknob_value
   Read_buttons
   Show_status

   Select Case Operationmode

      '
      'Switch to manual driving
      '
      Case 0:

         ' Wait for speed knob to return to zero
         Do
            Show_status
            Get_speedknob_value
         Loop Until Speedknobvalue = 0
         Operationmode = 1

      '
      'Manual Driving
      '
      Case 1:

         Delayup = 12
         Delaydown = 5
         Targetspeed = Speedknobvalue

         ' Switch to Automatic Driving
         If Buttons.1 = 0 Then
            Stopmotorandwaitforbuttons
            Operationmode = 2                               'Automatic Driving
            State = 1
         End If

         ' Switch Active Motor
         If Buttons.0 = 0 Then
            Switch_active_motor
            Stopmotorandwaitforbuttons
            Operationmode = 0
         End If

      '
      'Automatic Driving
      '
      Case 2:

'         ' Emergency stop and switch to manual driving
'         If Buttons.0 = 0 Then
'            Stopmotorandwaitforbuttons
'            Operationmode = 0                               'Manual Driving
'         End If

         ' Switch to manual driving
         If Buttons.1 = 0 Then
            Stopping = 1
         End If


         Delayup = 20
         Delaydown = 9

         Select Case State
            '
            'Stopping
            '
            Case 0:
'[ml]          Rounds = 2
               Targetspeed = 0
               If Motorspeed = 0 Then
                  Switch_active_motor
                  State = 1
                  Tmr1 = 0
               End If

            '
            'Stopped
            '
            Case 1:
               Rounds = 2
               Targetspeed = 0
               If Tmr1 > 6000 Then                          '3 seconds
'                 If Tmr1 > 8000 Then                        '4 seconds
                  State = 2
                  Tmr2 = 0
               End If

            '
            'Driving
            '
            Case 2:
               Targetspeed = Speedknobvalue

               ' Limit driving speed after 4 seconds
'                 W = Targetspeed * 28
'                 W = 19500 - W
'                 If Tmr2 >= W Then                         '
'                    Tmr2 = W
'                    W = Targetspeed
'                    If W > 60 Then
'                       If Rounds <= 1 Then
'                          W = W / 2
'                       Else
'                          W = W * 2
'                          W = W / 3
'                       End If
'                       If W < 60 Then
'                          W = 60
'                       End If
'                    End If
'                    Targetspeed = W
'                 Else
'                    W = W - 3000
'                    If Tmr2 >= W Then
'                       W = Targetspeed
'                       If W > 60 Then
'                          W = W * 2
'                          W = W / 3
'                          If W < 60 Then
'                             W = 60
'                          End If
'                       End If
'                       Targetspeed = W
'                    End If
'                 End If

               W = Targetspeed * 28
               W = 19500 - W
               If Tmr2 >= W Then                            '
                  Tmr2 = W
                  W = Targetspeed
                  If W > 140 Then
                     W = 140
                  End If
                  Targetspeed = W
               Else
                  W = W - 3000
                  If Tmr2 >= W Then
                     W = Targetspeed
                     If W > 140 Then
                        W = 140
                     End If
                     Targetspeed = W
                  End If
               End If
               If Activemotor = 1 Then
                  Channel = 3
               Else
                  Channel = 2
               End If

               ' Check light sensor
               W = Getadc(channel)
               If W > 280 Then
                  Do
                     Control_motor_speed
                     W = Getadc(channel)
                  Loop Until W < 280

                  Decr Rounds
                  If Rounds = 0 Then
                     State = 0
                  End If

                  'Aready driving, so start at 2000
                  Tmr2 = 2000

                  ' Switch to manual driving when we are stopped
                  If Stopping = 1 Then
                     Stopping = 0
                     Stopmotorandwaitforbuttons
                     Operationmode = 0                      'Manual Driving
                  End If

               End If

         End Select


   End Select

   ' Adjust motor speed to target speed
   If Motorspeed < Targetspeed Then
      If Tmr1 >= Delayup Then
         Incr Motorspeed
         Tmr1 = 0
      End If
   Elseif Motorspeed > Targetspeed Then
      If Tmr1 >= Delaydown Then
         Decr Motorspeed
         Tmr1 = 0
      End If
   End If

Loop



Sub Control_motor_speed

   If Activemotor = 1 Then
      W = Getadc(1)
   Else
      W = Getadc(0)
   End If
   Shift W , Right , 2

   If W < Motorspeed Then
      If Motorpwm < 255 Then
         Incr Motorpwm
      End If
   End If
   If W > Motorspeed Then
      If Motorpwm > 0 Then
         Decr Motorpwm
      End If
   End If

   ' Set new PWM value
   Pwm1a = Motorpwm

   ' Pinb.0:  Input=Motor1 Enabled;  Output 1=Disabled
   ' Pinb.2:  Input=Motor2 Enabled;  Output 1=Disabled
   ' Portb.0=1 and Portb.2=1 already set during initialisation
'    Reset Ddrb.0
'    Reset Ddrb.2

    Set Portb.0
    Set Portb.2
    If Activemotor = 1 Then
       Reset Ddrb.0
       Set Ddrb.2
    Elseif Activemotor = 2 Then
       Set Ddrb.0
       Reset Ddrb.2
    Else
       Set Ddrb.0
       Set Ddrb.2
    End If

End Sub


Sub Get_speedknob_value()

   W = Getadc(4)

   '
   '  convert range 0..1023 to 0..255
   '
   W = W / 3
   If W > 42 Then
      W = W - 42
   Else
      W = 0
   End If
   If W > 255 Then
      W = 255
   End If
   Speedknobvalue = W

End Sub


Sub Read_buttons()
   Buttons = Pind And &B11000000
   Shift Buttons , Right , 6
End Sub


Sub Show_status()
   Set Portd.0
   Set Portd.1
   Set Portd.2
   If Operationmode = 2 Then
      If Stopping = 0 Or Tmr1.7 = 0 Then
         Reset Portd.2
      End If
   End If

   ' Blink led when waiting for knob to return to zero
   If Operationmode = 0 Then
      If Tmr1.7 = 0 Then
         Exit Sub
      End If
   End If

   ' Activate leds
   If Activemotor = 1 Then
      Reset Portd.0
   Elseif Activemotor = 2 Then
      Reset Portd.1
   End If
End Sub


Sub Stopmotorandwaitforbuttons

   ' Stop Motors
   Targetspeed = 0
   Motorspeed = 0
   Motorpwm = 0
   Control_motor_speed

   Waitms 25

   ' Wait for buttons to be released
   Do
      Show_status
      Read_buttons
   Loop Until Buttons = &B00000011

   Waitms 25

End Sub

Sub Switch_active_motor
   If Activemotor = 2 Then
      Activemotor = 1
   Else
      Activemotor = 2
   End If
End Sub

Sub My_getadc(byval Channel As Byte)
   W = Getadc(channel)
   Exit Sub
   Admux = Channel
$asm
   sbi  adcsr,6
Adcloop1:
   sbic adcsr,6
   rjmp adcloop1
   sbi  adcsr,6
Adcloop2:
   sbic adcsr,6
   rjmp adcloop2

   in   R24,adcl
   in   R25,adch
   Loadadr W , X
   st   x+,R24
   st   x,R25
$end Asm
End Sub

Timer0_isr:
   Incr Tmr1
   Incr Tmr2
Return