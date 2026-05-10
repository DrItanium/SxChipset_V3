; this expert system computes the percentage breakdown of the different i960
; routines based on observational data acquired during runtime (in teensy cycles).
; The biggest takeaway is that signalling and waiting for ready takes over 50%
; of the overall time. Each ready signal requires 300 cycles worth of waiting
; time
(deftemplate execution-breakdown
             (multislot components
                        (type INTEGER)
                        (range 0 ?VARIABLE)
                        (default ?NONE))
             (slot total
                   (type SYMBOL
                         INTEGER)
                   (range 0 ?VARIABLE)
                   (default FALSE))
             (multislot percentages
                        (type NUMBER))
             (slot description
                   (type LEXEME)
                   (default ?NONE)))

(defrule compute-total
         ?f <- (execution-breakdown (total FALSE)
                                    (components $?elements))
         =>
         (modify ?f
                 (total (+ (expand$ ?elements)))))
(defrule compute-percentages
         ?f <- (execution-breakdown (total ?total&~FALSE)
                                    (components $?elements)
                                    (percentages))
         =>
         (bind ?cents
               (create$))
         (progn$ (?item ?elements)
                 (bind ?cents
                       ?cents
                       (/ ?item
                          ?total)))
         (modify ?f 
                 (percentages ?cents)))

(deffacts worst-case
          (execution-breakdown (description "stos")
                               (components 400 300 240))
          (execution-breakdown (description "stob")
                               (components 400 300 200))
          (execution-breakdown (description "st")
                               (components 400 
                                           (* 300 2) 
                                           480))
          (execution-breakdown (description "stl")
                               (components 400
                                           (* 300 4)
                                           960))
          (execution-breakdown (description "stt")
                               (components 400
                                           (* 300 6)
                                           1440))
          (execution-breakdown (description "stq")
                               (components 400
                                           (* 300 8)
                                           1920)))



                               
