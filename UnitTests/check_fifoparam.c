/*
 * Check: a unit test framework for C
 * Copyright (C) 2001, 2002 Arien Malec
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <config.h>
#include <stdlib.h>
#include <stdint.h>
#include <check.h>
#include "../bsp/fifoparam.h"

#define FIRST_QUE_ADDRESS 82
#define FIRST_QUE_DATA 2022
#define LOOP_ADDRESS_VALUE 21
#define LOOP_DATA_VALUE 65834
#define FIFO_HELL 666

/*
Introduction on writing and configuring testing protocol
Overview How Check Works:
    Check uses the following terms.
    Test fuction: A simple function that typically uses ck_assert_* functions
    test Cases: A collection of 'test functions'
    test Suit: A collection of Test Cases

*/

FIFO_PARAM_TYPE Test_FIFO;

// Code will run after each unit test 
void setup(void)
{
    FifoParamInit(&Test_FIFO);
}

// this code will help us delte the data in Test_FIFP
void teardown(void)
{
    Test_FIFO.usFifoParamBufferHead = 0;
    Test_FIFO.usFifoParamBufferTail = 0;

    //go ahead and loop around the fifo and set all values to zero
    for( int i = 0; i < FIFO_PARAM_BUFFER_SIZE; i++)
    {
        Test_FIFO.asFifoParamBuffer[i].usAddress = 0;
        Test_FIFO.asFifoParamBuffer[i].usData = 0;
    }

}


/*
    START_TEST marco can be thought of as a function that takes 
    as an arguements the name of our test function in this case test_FIFO_init
    Then using the assert function we can test whether certain functions
    worked properly in this Test we check if we que the fifo properly
*/
START_TEST(test_FIFO_QUE)
{
    //form up a fifo entry and set a value to it
    //this will also be the Hell Param value 
    FIFO_PARAM_BUFFER_TYPE sParam;
    FIFO_PARAM_BUFFER_TYPE HellParam;    

    //setup a CheckParam buffer value 
    FIFO_PARAM_TYPE CheckParam;

    //form up write fifo entry
    //the data is not used so its set to zero.
    sParam.usAddress = FIRST_QUE_ADDRESS;
    sParam.usData = FIRST_QUE_DATA;

    //setup a Hell Data 
    HellParam.usAddress = FIFO_HELL;
    HellParam.usData = FIFO_HELL;
    
    FifoParamQue(&Test_FIFO, sParam);
    ck_assert_msg( (Test_FIFO.asFifoParamBuffer[0].usAddress == FIRST_QUE_ADDRESS), "Was expecting a Value of %d, but got %d", 
                    FIRST_QUE_ADDRESS, Test_FIFO.asFifoParamBuffer[0].usAddress );
    
    ck_assert_msg( (Test_FIFO.asFifoParamBuffer[0].usData == FIRST_QUE_DATA), "Was expecting a Value of %d, but got %d", 
                    FIRST_QUE_DATA, Test_FIFO.asFifoParamBuffer[0].usData );
    
    //Also Check that the Test_FIFO data also has this parameter inside its address
    //ck_assert_int_eq(Test_FIFO.asFifoParamBuffer[0].usAddress, FIRST_QUE_ADDRESS);
    //ck_assert_int_eq(Test_FIFO.asFifoParamBuffer[0].usData, FIRST_QUE_DATA);
    
    // Init you other check param  and key it with this data 
    FifoParamInit(&CheckParam);
    FifoParamQue(&CheckParam, HellParam);

    //Assert the same function and see if eqaulity occurs
    sParam.usAddress = LOOP_ADDRESS_VALUE;
    sParam.usData = LOOP_DATA_VALUE;
    
    for(int i=0; i<=FIFO_PARAM_BUFFER_SIZE; i++ ){
        //put data into the Test FIFO buffer 
         FifoParamQue(&CheckParam, sParam);
    }
    
    // attemp to add for data to this patameter 
    FifoParamQue(&CheckParam, HellParam);

    //FIFO_PARAM_SHOULD BE FULL check param at its last index will be the SParam
    ck_assert_int_ne( CheckParam.asFifoParamBuffer[CheckParam.usFifoParamBufferHead].usData, FIFO_HELL); 
    ck_assert_int_ne( CheckParam.asFifoParamBuffer[CheckParam.usFifoParamBufferHead].usAddress, FIFO_HELL); 
    
}
END_TEST


START_TEST(test_FIFO_DEQUE)
{
    //form up a fifo entry and set a value to it
    FIFO_PARAM_BUFFER_TYPE sParam;
    FIFO_PARAM_BUFFER_TYPE sSendingParam;
    
    //Sending the last index of the param here 
    sSendingParam.usAddress = 123;
    sSendingParam.usData = 987;
    FifoParamQue(&Test_FIFO, sSendingParam);

    //Assert the same function and see if eqaulity occurs
    sSendingParam.usAddress = LOOP_ADDRESS_VALUE;
    sSendingParam.usData = LOOP_DATA_VALUE;
    
    for(int i=0; i<=FIFO_PARAM_BUFFER_SIZE; i++ ){
        //put data into the Test FIFO buffer 
        FifoParamQue(&Test_FIFO, sSendingParam);
    }
    
    //run the deque function and get the selected values back from sParam
    FifoParamDeque( (&Test_FIFO) , (&sParam) );
        
    ck_assert_msg( (sParam.usAddress == 123), "Was expecting a Value of %d, but got %d",
                    123, sParam.usAddress );
    
    //verify that we get the wanted values from QUED Data from a different test
    ck_assert_msg( (sParam.usData == 987), "Was expecting a Value of %d, but got %d", 
                    987, sParam.usData );
    
    
}
END_TEST

/*
IN ORDER TO RUN THE TEST FUNCTIONS WE NEED TO ADD THEM TO THE TEST CASES ONCE WE 
HAVE DONE THIS THEN WE ADD THEM TO THE SUIT THIS IS THE CODING FUNCTIONALITY 
NEEDED IN ORDER TO MAKE THIS HAPPEN
THE C FUNCTION FIFO_SUITE TAKES NO ARGUEMENTS AND RETURNS A POINTER TO A SUIT
*/
Suite * FIFO_suite(void) 
{
    Suite *s;  //DECLARES S AS A POINTER OF THE SUIT 
    TCase *tc_core; //DECLARES TC_CORE AS THE POINTER TO THE TEST CASE
    TCase *tc_limits; //DECLARES THE TEST LIMITS TO THE POINTER OF TC_LIMITS

    //CREATE A NEW SUIT USING THE FUNCTION SUIT_CREATE AND GIVE THE SUIT THE NAME MONEY
    s = suite_create("FIFO"); 

    /* Core test case  ADDED THE TEST CASE USEING THE FUNCTION TCASE AND GIVEN THE FUNCTION NAME CORE*/
    tc_core = tcase_create("Core");

    //Add a ficture setup and teadrdown function to the test case these will be run before and after unit test
    tcase_add_checked_fixture(tc_core, setup, teardown);
    
    //Marco function that add a test function to a test case
    tcase_add_test(tc_core, test_FIFO_QUE); //ading test function "Test_FIFO_init"  
    //(s, tc_core);
    
    /* Limits test case */
    //tc_limits = tcase_create("Limits");
    tcase_add_test(tc_core, test_FIFO_DEQUE); //ading test function "test_FIFO_DEQUE"
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    //DECLARE AN INT VARIABLE TO STORE THE NUMBER OF FAILED TEST
    int number_failed;
    
    //DECLARE BUT NOT CREATE A POINTER TO A SUIT AND POINTER TO THE SUIT RUNNER
    Suite *s; 
    SRunner *sr;

    //CALL OUR FUNCTION FIFO_SUITE IN ORDER TO CREATE AND RETURN OUR TEST SUITE
    s = FIFO_suite();
    
    // We then use our newly created test suite and pass it to the function srunner_create
    //  in order to obtain back a pointer to a test suite runner.
    sr = srunner_create(s);

    // USING THE COMMAND: SRUNNER_RUN_ALL WE CAN PRINT ALL FAILED TEST 
    //SO THAT CHECK WILL RUN OUR TEST SUIT AND CHECK OUR ASSERTIONS
    srunner_run_all(sr, CK_VERBOSE);
    
    // GET THE NUMBER OF FAILED TEST FROM THE RUNNER THAT WAS PASSED
    number_failed = srunner_ntests_failed(sr);
    
    // FREE ANY MEMORY THAT WAS ALLOCATED DUE TO OUR RUNNER
    srunner_free(sr);
    
    //RETUN THE NUMBER OF EXIT_SUCCESS OR EXIT_FAILURE BASED ON NUMBER FAILED VARIABLE
    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}