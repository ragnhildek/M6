\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  751010.75 lambda_05 + 1.057911625e+06 lambda_16 + 746025.9375 lambda_27
   + 745894.375 lambda_38 + 92434.765625 lambda_09
   + 399069.21875 lambda_110 + 87483.3359375 lambda_211
   + 87351.8203125 lambda_312 + 652847.8125 lambda_013
   + 959548.125 lambda_114 + 650191.6875 lambda_215
   + 649725.9375 lambda_316 + 1898.693115234375 lambda_017
   + 300819.09375 lambda_118
Subject To
 oneVisitCon0: lambda_01 + lambda_05 + lambda_09 + lambda_013 + lambda_017
   = 1
 oneVisitCon1: lambda_12 + lambda_16 + lambda_110 + lambda_114 + lambda_118
   = 1
 oneVisitCon2: lambda_23 + lambda_27 + lambda_211 + lambda_215 = 1
 oneVisitCon3: lambda_34 + lambda_38 + lambda_312 + lambda_316 = 1
 visitedPickupCon0: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_013 + lambda_114 + lambda_215 + lambda_316 <= 1
 visitedPickupCon1: lambda_05 + lambda_16 + lambda_09 + lambda_110
   + lambda_013 + lambda_114 + lambda_017 + lambda_118 <= 1
 visitedPickupCon2: lambda_16 + lambda_110 + lambda_114 + lambda_118 <= 1
 visitedPickupCon3: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 <= 1
 visitedPickupCon4: <= 1
Bounds
End
