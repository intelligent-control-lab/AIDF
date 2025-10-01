;Header and description

(define (domain skillgraph)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot - object
    block - object

)

; un-comment following line if constants are needed
;(:constants )

(:predicates 
    (above ?robot - robot  ?block - block) ; hand is above block
    (under ?robot - robot ?block - block) ; hand is under block
    (inhand ?robot - robot ?block - block) ; hand is holding block
    (handempty ?robot) ; hand is empty
    (coverup ?block1 - block) ; block is covered by another block
    (coverdown ?block1 -block) ; block is covered by another block on bottom
    (on ?block1 - block ?block2 - block) ; block1 is on top of block2
    (presson ?robot - robot ?block - block) ; robot is pressing on block
    (supportbottom ?robot - robot ?block - block) ; robot is supporting block on
)


(:action pickup
    :parameters (?robot - robot ?block - block)
    :precondition (and 
        (above ?robot ?block) 
        (handempty ?robot)
        (not (coverup ?block) )
    )
    :effect (and 
        (not (handempty ?robot)) 
        (inhand ?robot ?block)
    )
)



(:action placetop
    :parameters (?robot - robot ?block1 - block ?block2 - block)
    :precondition (and 
        (inhand ?robot ?block1) 
        (above ?robot ?block2) 
        (not (coverup ?block2))
    )
    :effect (and 
        (not (inhand ?robot ?block1)) 
        (handempty ?robot) 
        (coverup ?block2) 
        (above ?robot ?block1)
        (on ?block1 ?block2)
    )
)


(:action placebottom
    :parameters (?robot - robot ?block1 - block ?block2 - block)
    :precondition (and 
        (inhand ?robot ?block1) 
        (under ?robot ?block2) 
        (not (coverdown ?block2))
    )
    :effect (and 
        (not (inhand ?robot ?block1)) 
        (handempty ?robot) 
        (coverdown ?block2) 
        (under ?robot ?block1)
        (on ?block2 ?block1)
    )
)

(:action supportbottom
    :parameters (?robot - robot ?block1 - block)
    :precondition (and 
        (handempty ?robot)
        (not (coverdown ?block1))
    )
    :effect (and 
        (coverdown ?block1) 
        (supportbottom ?robot ?block1)
    )
)

(:action supporttop
    :parameters (?robot - robot ?block1 - block)
    :precondition (and 
        (handempty ?robot)
        (not (coverup ?block1))
    )

    :effect (and 
        (coverup ?block1) 
        (presson ?robot ?block1)
    )
)


(:action handover
    :parameters (?robot1 - robot ?robot2 - robot ?block - block)
    :precondition (and 
        (inhand ?robot1 ?block) 
        (handempty ?robot2)
        (under ?robot2 ?block) ; robot2 is under the block
       
    )
    :effect (and 
        (not (inhand ?robot1 ?block)) 
        (handempty ?robot1) 
        (inhand ?robot2 ?block) 
        (not (under ?robot2 ?block)) ; robot2 is no longer under the block
        (above ?robot1 ?block) ; robot2 is now above the block
    )
)



;define actions here

)
