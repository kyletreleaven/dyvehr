



"""
Schema should include:
Experiment
    -ID            ID
    -arrivalrate    real
    -numveh        int
    -vehspeed        real
    -distrib_id    ID
    -utilization    real    (redundant)
    -policy_id    ID
    #
    -init_dur    real
    -time_factor    real
    -thresh_factor    real
    -exploit_ratio    real
    
Distribution
    -ID            ID
    -descr        text
    -meancarry    real
    -meanfetch    real
    -moverscplx    real    (redundant)


Policies
    -ID            ID
    -descr        text
    
    
Demands
    -local_id          ID
    -experiment_id    ID
    PRIMARY KEY (experiment_id,local_id)
    #
    -arrivaltime        real
    -embarktime        real
    -deliverytime    real
    -waitdur        real    (redundant)
    -carrydur        real (redundant)
    -systemdur        real (redundant)
    
*We can manage this with a gui, yeah?
"""






