\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  711240.9597036 lambda_0_4 + 710739.9597036 lambda_0_5
   + 710238.9597036 lambda_0_6 + 703892.9597036 lambda_0_7
   + 703391.9597036 lambda_0_8 + 24086.8002596 lambda_0_9
   + 23585.8002596 lambda_0_10 + 244717.9898912 lambda_1_11
   + 244216.9898912 lambda_1_12 + 243715.9898912 lambda_1_13
   + 237369.9898912 lambda_1_14 + 236868.9898912 lambda_1_15
   + 152931.8951756001 lambda_1_16 + 152430.8951756001 lambda_1_17
   + 145583.8951756001 lambda_1_18 + 51068.2062692 lambda_1_19
   + 50567.2062692 lambda_1_20 + 50066.2062692 lambda_1_21
   + 49565.2062692 lambda_1_22 + 43720.2062692 lambda_1_23
   + 43219.2062692 lambda_1_24 + 713016.3916956001 lambda_2_25
   + 712515.3916956001 lambda_2_26 + 712014.3916956001 lambda_2_27
   + 710377.6837712001 lambda_3_28 + 709876.6837712001 lambda_3_29
   + 709375.6837712001 lambda_3_30 + 703029.6837712001 lambda_3_31
   + 702528.6837712001 lambda_3_32 + 24719.6469448 lambda_3_33
   + 24218.6469448 lambda_3_34 + 23717.6469448 lambda_3_35
   + 18821.2054316 lambda_3_36 + 18320.2054316 lambda_3_37
   + 17819.2054316 lambda_3_38
Subject To
 oneVisitCon0: lambda_0_0 + lambda_0_4 + lambda_0_5 + lambda_0_6
   + lambda_0_7 + lambda_0_8 + lambda_0_9 + lambda_0_10 = 1
 oneVisitCon1: lambda_1_1 + lambda_1_11 + lambda_1_12 + lambda_1_13
   + lambda_1_14 + lambda_1_15 + lambda_1_16 + lambda_1_17 + lambda_1_18
   + lambda_1_19 + lambda_1_20 + lambda_1_21 + lambda_1_22 + lambda_1_23
   + lambda_1_24 = 1
 oneVisitCon2: lambda_2_2 + lambda_2_25 + lambda_2_26 + lambda_2_27 = 1
 oneVisitCon3: lambda_3_3 + lambda_3_28 + lambda_3_29 + lambda_3_30
   + lambda_3_31 + lambda_3_32 + lambda_3_33 + lambda_3_34 + lambda_3_35
   + lambda_3_36 + lambda_3_37 + lambda_3_38 = 1
 visitedPickupCon0: lambda_0_4 + lambda_0_5 + lambda_0_6 + lambda_0_7
   + lambda_0_8 + lambda_2_25 + lambda_2_26 + lambda_2_27 + lambda_3_28
   + lambda_3_29 + lambda_3_30 + lambda_3_31 + lambda_3_32 <= 1
 visitedPickupCon1: lambda_1_11 + lambda_1_12 + lambda_1_13 + lambda_1_14
   + lambda_1_15 <= 1
 visitedPickupCon2: lambda_0_9 + lambda_0_10 + lambda_1_19 + lambda_1_20
   + lambda_1_21 + lambda_1_22 + lambda_1_23 + lambda_1_24 + lambda_3_33
   + lambda_3_34 + lambda_3_35 <= 1
 visitedPickupCon3: lambda_1_19 + lambda_1_20 + lambda_1_21 + lambda_1_22
   + lambda_1_23 + lambda_1_24 + lambda_3_36 + lambda_3_37 + lambda_3_38
   <= 1
 visitedPickupCon4: lambda_1_16 + lambda_1_17 + lambda_1_18 <= 1
Bounds
End
