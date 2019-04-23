\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  110410.609375 lambda_05 + 111798.4140625 lambda_16
   + 238285.265625 lambda_27 + 238285.265625 lambda_38
   + 107059.6328125 lambda_09 + 108728.4921875 lambda_110
   + 125323.53125 lambda_211 + 125323.53125 lambda_312
   + 18446.935546875 lambda_013 + 20634.208984375 lambda_114
   + 20837.228515625 lambda_215 + 20837.228515625 lambda_316
Subject To
 oneVisitCon0: lambda_01 + lambda_05 + lambda_09 + lambda_013 = 1
 oneVisitCon1: lambda_12 + lambda_16 + lambda_110 + lambda_114 = 1
 oneVisitCon2: lambda_23 + lambda_27 + lambda_211 + lambda_215 = 1
 oneVisitCon3: lambda_34 + lambda_38 + lambda_312 + lambda_316 = 1
 visitedPickupCon0: <= 1
 visitedPickupCon1: <= 1
 visitedPickupCon2: <= 1
 visitedPickupCon3: lambda_013 + lambda_114 + lambda_215 + lambda_316 <= 1
 visitedPickupCon4: lambda_27 + lambda_38 <= 1
 visitedPickupCon5: lambda_09 + lambda_110 <= 1
 visitedPickupCon6: lambda_211 + lambda_312 <= 1
 visitedPickupCon7: <= 1
 visitedPickupCon8: lambda_05 + lambda_16 <= 1
 visitedPickupCon9: <= 1
Bounds
End
