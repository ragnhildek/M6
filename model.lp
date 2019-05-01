\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  819248.407858 lambda_0_4 + 902011.2850624 lambda_1_5
   + 665630.3132620003 lambda_2_6 + 921040.5094924001 lambda_3_7
   + 606160.0298363997 lambda_0_8 + 816246.6332820002 lambda_1_9
   + 882802.4371039998 lambda_3_10 + 594307.6084584 lambda_0_11
   + 801819.0252104002 lambda_1_12 + 878614.5743228002 lambda_3_13
   + 554387.5239251998 lambda_0_14 + 704126.2698880002 lambda_1_15
   + 806505.1019888 lambda_3_16 + 542242.4379248 lambda_0_17
   + 663595.5364652001 lambda_1_18 + 489272.5919024 lambda_2_19
   + 636800.0368379999 lambda_3_20 + 556460.9989807999 lambda_0_21
   + 732602.8815488 lambda_1_22 + 842549.4658807999 lambda_3_23
   + 515665.9819868 lambda_0_24 + 563843.762592 lambda_1_25
   + 491957.3856087999 lambda_3_26 + 553989.0065371997 lambda_0_27
   + 550077.6859256001 lambda_1_28 + 590767.9358271997 lambda_3_29
   + 788317.7768396003 lambda_1_30 + 579522.3672148 lambda_3_31
   + 536261.8657575999 lambda_1_32 + 857460.3217727998 lambda_3_33
   + 746640.8397292001 lambda_1_34
Subject To
 oneVisitCon0: lambda_0_0 + lambda_0_4 + lambda_0_8 + lambda_0_11
   + lambda_0_14 + lambda_0_17 + lambda_0_21 + lambda_0_24 + lambda_0_27
   = 1
 oneVisitCon1: lambda_1_1 + lambda_1_5 + lambda_1_9 + lambda_1_12
   + lambda_1_15 + lambda_1_18 + lambda_1_22 + lambda_1_25 + lambda_1_28
   + lambda_1_30 + lambda_1_32 + lambda_1_34 = 1
 oneVisitCon2: lambda_2_2 + lambda_2_6 + lambda_2_19 = 1
 oneVisitCon3: lambda_3_3 + lambda_3_7 + lambda_3_10 + lambda_3_13
   + lambda_3_16 + lambda_3_20 + lambda_3_23 + lambda_3_26 + lambda_3_29
   + lambda_3_31 + lambda_3_33 = 1
 visitedPickupCon0: lambda_0_24 + lambda_3_26 <= 1
 visitedPickupCon1: lambda_1_9 + lambda_1_12 + lambda_1_15 + lambda_1_18
   + lambda_3_20 + lambda_1_22 + lambda_1_30 + lambda_1_34 <= 1
 visitedPickupCon2: lambda_1_5 + lambda_3_7 + lambda_0_8 + lambda_1_9
   + lambda_0_11 + lambda_1_12 + lambda_3_13 + lambda_0_21 + lambda_1_22
   + lambda_3_23 + lambda_1_30 + lambda_1_34 <= 1
 visitedPickupCon3: lambda_0_8 + lambda_3_10 + lambda_0_11 + lambda_0_27
   + lambda_3_29 <= 1
 visitedPickupCon4: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_3_13
   + lambda_3_16 <= 1
 visitedPickupCon5: lambda_2_6 + lambda_0_14 <= 1
 visitedPickupCon6: lambda_2_6 + lambda_0_24 + lambda_3_26 + lambda_3_29
   + lambda_3_31 <= 1
 visitedPickupCon7: lambda_1_12 + lambda_1_18 + lambda_3_20 + lambda_1_25
   + lambda_1_28 + lambda_1_32 + lambda_1_34 <= 1
 visitedPickupCon8: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_1_9
   + lambda_1_12 + lambda_3_13 + lambda_1_15 + lambda_3_16 + lambda_1_30
   <= 1
 visitedPickupCon9: <= 1
 visitedPickupCon10: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_3_10
   + lambda_3_13 + lambda_3_16 + lambda_3_23 + lambda_3_33 <= 1
 visitedPickupCon11: lambda_1_9 + lambda_1_12 + lambda_1_15 + lambda_1_18
   + lambda_1_22 + lambda_0_24 + lambda_1_30 + lambda_1_34 <= 1
 visitedPickupCon12: lambda_2_6 + lambda_0_14 <= 1
 visitedPickupCon13: lambda_3_7 + lambda_3_10 + lambda_0_11 + lambda_3_16
   + lambda_0_21 + lambda_3_23 + lambda_1_28 <= 1
 visitedPickupCon14: lambda_0_8 + lambda_3_10 + lambda_0_11 + lambda_0_17
   + lambda_2_19 + lambda_0_27 + lambda_3_29 + lambda_3_31 + lambda_3_33
   <= 1
 visitedPickupCon15: lambda_2_6 <= 1
 visitedPickupCon16: lambda_1_25 + lambda_1_28 + lambda_1_32 <= 1
 visitedPickupCon17: lambda_0_21 + lambda_3_23 + lambda_0_24 + lambda_1_25
   + lambda_3_26 + lambda_1_28 + lambda_1_32 <= 1
 visitedPickupCon18: lambda_0_14 + lambda_0_17 + lambda_0_21 + lambda_0_27
   + lambda_3_29 + lambda_3_31 <= 1
 visitedPickupCon19: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_0_8
   + lambda_1_9 + lambda_3_10 + lambda_0_24 + lambda_1_25 + lambda_3_26
   + lambda_3_33 <= 1
Bounds
End
