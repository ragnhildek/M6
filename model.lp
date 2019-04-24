\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  429413.3886683999 lambda_0_5 + 695423.0806992 lambda_1_6
   + 466811.634596 lambda_2_7 + 695423.0806992 lambda_3_8
   + 228029.1444864 lambda_0_9 + 228186.7482940001 lambda_1_10
   + 97323.63937000002 lambda_2_11 + 228186.7482940001 lambda_3_12
   + 95567.47723840001 lambda_0_13 + 95494.66025440001 lambda_1_14
   + 95494.66025440001 lambda_3_15 + 68371.17690399999 lambda_0_16
   + 68528.7807116 lambda_1_17 + 68528.7807116 lambda_3_18
   + 157473.3711552 lambda_0_19 + 154153.8620872 lambda_1_20
   + 154153.8620872 lambda_3_21
Subject To
 oneVisitCon0: lambda_0_1 + lambda_0_5 + lambda_0_9 + lambda_0_13
   + lambda_0_16 + lambda_0_19 = 1
 oneVisitCon1: lambda_1_2 + lambda_1_6 + lambda_1_10 + lambda_1_14
   + lambda_1_17 + lambda_1_20 = 1
 oneVisitCon2: lambda_2_3 + lambda_2_7 + lambda_2_11 = 1
 oneVisitCon3: lambda_3_4 + lambda_3_8 + lambda_3_12 + lambda_3_15
   + lambda_3_18 + lambda_3_21 = 1
 visitedPickupCon0: lambda_0_9 + lambda_1_10 + lambda_3_12 + lambda_0_19
   + lambda_1_20 + lambda_3_21 <= 1
 visitedPickupCon1: lambda_0_5 + lambda_2_7 <= 1
 visitedPickupCon2: lambda_2_7 + lambda_2_11 + lambda_0_13 + lambda_1_14
   + lambda_3_15 <= 1
 visitedPickupCon3: lambda_0_5 + lambda_0_9 + lambda_1_10 + lambda_3_12
   + lambda_0_16 + lambda_1_17 + lambda_3_18 <= 1
 visitedPickupCon4: lambda_1_6 + lambda_3_8 <= 1
Bounds
End
