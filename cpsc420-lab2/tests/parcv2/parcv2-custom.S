//========================================================================
// parcv2-custom.s
//========================================================================

#include "parc-macros.h"

        TEST_PARC_BEGIN

        //-----------------------------------------------------------------
        // X → D bypass: pure ALU-to-ALU dependency
        //
        //   addiu $2,$0,10     ; $2 = 10
        //   addiu $3,$0,5      ; $3 = 5
        //   addu  $1,$2,$3     ; $1 = 15   (producer in X)
        //   addu  $4,$1,$3     ; $4 = 20   (consumer in D, needs X→D bypass)
        //-----------------------------------------------------------------

        addiu $2, $0, 10
        addiu $3, $0, 5
        addu  $1, $2, $3
        addu  $4, $1, $3

        TEST_CHECK_EQ( $4, 20 )

        //-----------------------------------------------------------------
        // M → D / W → D style reuse of the same producer
        //
        // Keep using $1 (15) as a source in back-to-back instructions.
        // Each consumer sits behind the previous one in the pipe, so over
        // time it will see the value of $1 as it flows through X/M/W and
        // must be forwarded correctly at each point.
        //-----------------------------------------------------------------

        addu  $5, $1, $3 // expected 20
        TEST_CHECK_EQ( $5, 20 )

        addu  $6, $1, $3 // expected 20
        TEST_CHECK_EQ( $6, 20 )

        //-----------------------------------------------------------------
        // Load-use hazard + M-bypass on load result
        //
        //   addiu $7,$0,100     ; base addr
        //   sw    $2,0($7)      ; store 10
        //   lw    $8,0($7)      ; load -> load-use hazard
        //   addu  $9,$8,$3      ; 10 + 5 = 15, must see loaded data
        //
        // Our design will:
        //   - stall one cycle for the load-use (X load / D consumer),
        //   - then use M/W bypass path so $8’s value feeds $9.
        //-----------------------------------------------------------------

        addiu $7, $0, 100
        sw    $2, 0($7)
        lw    $8, 0($7)
        addu  $9, $8, $3

        TEST_CHECK_EQ( $9, 15 )

        TEST_PARC_END
