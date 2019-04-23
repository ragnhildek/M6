\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  746826.1875 lambda_05 + 929546.6875 lambda_16 + 928957.8125 lambda_27
   + 928957.8125 lambda_38 + 737534.5625 lambda_09 + 903572.375 lambda_110
   + 903270.8125 lambda_211 + 903270.8125 lambda_312
   + 481439.28125 lambda_013 + 600470.5 lambda_114 + 599881.6875 lambda_215
   + 599881.6875 lambda_316 + 430322.9375 lambda_017 + 513333 lambda_118
   + 473375.4375 lambda_219 + 473375.4375 lambda_320
   + 356019.625 lambda_021 + 514486.21875 lambda_122
   + 514184.625 lambda_223 + 514184.625 lambda_324
   + 335288.84375 lambda_025 + 472669.25 lambda_126
   + 469155.9375 lambda_227 + 469155.9375 lambda_328
   + 438585.625 lambda_029 + 451605.46875 lambda_130
   + 456868.0625 lambda_231 + 456868.0625 lambda_332
   + 412626.8125 lambda_033 + 837888 lambda_134 + 450066.03125 lambda_235
   + 450066.03125 lambda_336 + 590077.0625 lambda_237
   + 590077.0625 lambda_338
Subject To
 oneVisitCon0: lambda_01 + lambda_05 + lambda_09 + lambda_013 + lambda_017
   + lambda_021 + lambda_025 + lambda_029 + lambda_033 = 1
 oneVisitCon1: lambda_12 + lambda_16 + lambda_110 + lambda_114 + lambda_118
   + lambda_122 + lambda_126 + lambda_130 + lambda_134 = 1
 oneVisitCon2: lambda_23 + lambda_27 + lambda_211 + lambda_215 + lambda_219
   + lambda_223 + lambda_227 + lambda_231 + lambda_235 + lambda_237 = 1
 oneVisitCon3: lambda_34 + lambda_38 + lambda_312 + lambda_316 + lambda_320
   + lambda_324 + lambda_328 + lambda_332 + lambda_336 + lambda_338 = 1
 visitedPickupCon0: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_021 + lambda_122 + lambda_223 + lambda_324 + lambda_033 <= 1
 visitedPickupCon1: lambda_013 + lambda_114 + lambda_215 + lambda_316
   + lambda_029 + lambda_231 + lambda_332 <= 1
 visitedPickupCon2: lambda_025 <= 1
 visitedPickupCon3: lambda_09 + lambda_114 + lambda_215 + lambda_316
   + lambda_130 + lambda_134 + lambda_237 + lambda_338 <= 1
 visitedPickupCon4: lambda_114 + lambda_215 + lambda_316 + lambda_017
   + lambda_029 + lambda_231 + lambda_332 + lambda_033 + lambda_235
   + lambda_336 + lambda_237 + lambda_338 <= 1
 visitedPickupCon5: lambda_114 + lambda_215 + lambda_316 + lambda_017
   + lambda_029 + lambda_231 + lambda_332 + lambda_033 + lambda_235
   + lambda_336 + lambda_237 + lambda_338 <= 1
 visitedPickupCon6: lambda_16 + lambda_27 + lambda_38 + lambda_110
   + lambda_211 + lambda_312 + lambda_114 + lambda_215 + lambda_316
   + lambda_122 + lambda_223 + lambda_324 + lambda_237 + lambda_338 <= 1
 visitedPickupCon7: lambda_118 + lambda_126 + lambda_227 + lambda_328 <= 1
 visitedPickupCon8: lambda_017 + lambda_118 + lambda_029 <= 1
 visitedPickupCon9: lambda_013 + lambda_219 + lambda_320 <= 1
 visitedPickupCon10: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 + lambda_134 <= 1
 visitedPickupCon11: lambda_231 + lambda_332 + lambda_235 + lambda_336
   <= 1
 visitedPickupCon12: lambda_013 + lambda_219 + lambda_320 + lambda_021
   + lambda_122 + lambda_223 + lambda_324 + lambda_130 <= 1
 visitedPickupCon13: lambda_130 + lambda_134 <= 1
 visitedPickupCon14: lambda_05 + lambda_16 + lambda_27 + lambda_38
   + lambda_09 + lambda_110 + lambda_211 + lambda_312 + lambda_021 <= 1
Bounds
End
