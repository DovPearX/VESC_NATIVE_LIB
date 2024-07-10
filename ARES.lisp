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

     (define is_wheelslip   (ext-arc-debug  14))
     (define highaccelon    (ext-arc-debug  15))
     (define debug1         (ext-arc-debug  16))
     (define debug2         (ext-arc-debug  17))
     (define debug3         (ext-arc-debug  18))
     (define debug4         (ext-arc-debug  19))
     (define debug5         (ext-arc-debug  20))
     (define debug6         (ext-arc-debug  21))
     (define debug7         (ext-arc-debug  22))
     (define debug8         (ext-arc-debug  23))
     (define debug9         (ext-arc-debug  24))
     ;(define aggregate_timer(ext-arc-debug  25))
     ;(define freq_factor    (ext-arc-debug  26))

     (sleep 0.1)
)))