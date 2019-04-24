\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  613559.75 lambda_0_5 + 613594.5625 lambda_1_6 + 614159 lambda_2_7
   + 614582.8125 lambda_3_8 + 406786.125 lambda_0_9
   + 407412.96875 lambda_1_10 + 360658.71875 lambda_2_11
   + 409739.65625 lambda_3_12 + 112208.109375 lambda_0_13
   + 112190.7734375 lambda_1_14 + 213417.3125 lambda_2_15
   + 208327.765625 lambda_3_16 + 356810.5625 lambda_0_17
   + 356846.6875 lambda_1_18 + 357893.84375 lambda_3_19
   + 113601.1171875 lambda_3_20
Subject To
 oneVisitCon0: lambda_0_1 + lambda_0_5 + lambda_0_9 + lambda_0_13
   + lambda_0_17 = 1
 oneVisitCon1: lambda_1_2 + lambda_1_6 + lambda_1_10 + lambda_1_14
   + lambda_1_18 = 1
 oneVisitCon2: lambda_2_3 + lambda_2_7 + lambda_2_11 + lambda_2_15 = 1
 oneVisitCon3: lambda_3_4 + lambda_3_8 + lambda_3_12 + lambda_3_16
   + lambda_3_19 + lambda_3_20 = 1
 visitedPickupCon0: lambda_0_5 + lambda_1_6 + lambda_2_7 + lambda_3_8 <= 1
 visitedPickupCon1: lambda_0_5 + lambda_1_6 + lambda_2_7 + lambda_3_8
   + lambda_2_11 + lambda_0_17 + lambda_1_18 + lambda_3_19 <= 1
 visitedPickupCon2: lambda_0_13 + lambda_1_14 + lambda_3_20 <= 1
 visitedPickupCon3: lambda_2_15 + lambda_3_16 <= 1
 visitedPickupCon4: lambda_0_9 + lambda_1_10 + lambda_3_12 <= 1
Bounds
End
