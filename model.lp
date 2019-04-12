\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  1.36802475e+06 lambda_05 + 1.36879825e+06 lambda_16
   + 1.368799e+06 lambda_27 + 1.3680165e+06 lambda_38
   + 1.360049e+06 lambda_09 + 1.3608225e+06 lambda_110
   + 1.36082325e+06 lambda_211 + 1.360040875e+06 lambda_312
Subject To
 oneVisitCon0: lambda_01 + lambda_05 + lambda_09 = 1
 oneVisitCon1: lambda_12 + lambda_16 + lambda_110 = 1
 oneVisitCon2: lambda_23 + lambda_27 + lambda_211 = 1
 oneVisitCon3: lambda_34 + lambda_38 + lambda_312 = 1
 visitedPickupCon0: lambda_05 + lambda_16 + lambda_27 + lambda_38 <= 1
 visitedPickupCon1: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 <= 1
 visitedPickupCon2: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 <= 1
 visitedPickupCon3: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 <= 1
 visitedPickupCon4: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 <= 1
Bounds
End
