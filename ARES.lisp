(import "/home/nobodyx/Pulpit/VESC/DEMETER/ARES/build/ARES.bin" 'arc)


(load-native-lib arc)

(loopwhile t
  (progn

     (define curr_time      (ext-arc-debug  1))
     (define gyrox          (ext-arc-debug  2))
     (define gyroy          (ext-arc-debug  3))
     (define gyroz          (ext-arc-debug  4))

     (define accelx         (ext-arc-debug  5))
     (define accely         (ext-arc-debug  6))
     (define accelz         (ext-arc-debug  7))

     (define speed        (ext-arc-debug  8))
     (define test2        (ext-arc-debug  9))
     (define test3        (ext-arc-debug  10))
     (define test4        (ext-arc-debug  11))


     (sleep 0.1)
)))