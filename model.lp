\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  360633.8125 lambda_05 + 361085.1875 lambda_16 + 360309.625 lambda_27
   + 360725.34375 lambda_38 + 190068.28125 lambda_09
   + 329930.5625 lambda_110 + 189747.46875 lambda_211
   + 190129.796875 lambda_312 + 175686.34375 lambda_013
   + 266405.1875 lambda_114 + 175328.75 lambda_215 + 176071.8125 lambda_316
Subject To
 oneVisitCon0: lambda_01 + lambda_05 + lambda_09 + lambda_013 = 1
 oneVisitCon1: lambda_12 + lambda_16 + lambda_110 + lambda_114 = 1
 oneVisitCon2: lambda_23 + lambda_27 + lambda_211 + lambda_215 = 1
 oneVisitCon3: lambda_34 + lambda_38 + lambda_312 + lambda_316 = 1
 visitedPickupCon0: lambda_09 + lambda_211 + lambda_312 <= 1
 visitedPickupCon1: lambda_110 <= 1
 visitedPickupCon2: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_013 + lambda_215 + lambda_316 <= 1
 visitedPickupCon3: lambda_114 <= 1
 visitedPickupCon4: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_211 + lambda_312 <= 1
Bounds
End
