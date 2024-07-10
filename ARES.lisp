(import "/home/nobodyx/Pulpit/VESC/DEMETER/ARES/build/ARES.bin" 'arc)


(load-native-lib arc)


(loopwhile t
  (progn

    ; (define curr_time      (ext-arc-debug  1))
    ; (define gyrox          (ext-arc-debug  2))
    ; (define gyroy          (ext-arc-debug  3))
    ; (define gyroz          (ext-arc-debug  4))

    ; (define accelx         (ext-arc-debug  5))
    ; (define accely         (ext-arc-debug  6))
    ; (define accelz         (ext-arc-debug  7))

     (define duty_cycle     (ext-arc-debug  8))
     (define current        (ext-arc-debug  9))
     (define braking        (ext-arc-debug  10))
     (define abs_erpm       (ext-arc-debug  11))
     (define acceleration   (ext-arc-debug  12))
     (define current_avg    (ext-arc-debug  13))


     (sleep 0.1)
)))