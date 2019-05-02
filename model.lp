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
   + 746640.8397292001 lambda_1_34 + 606160.0298363997 lambda_0_35
   + 902011.2850624 lambda_1_36 + 665630.3132620003 lambda_2_37
   + 921040.5094924001 lambda_3_38 + 459973.3961139999 lambda_0_39
   + 824191.0333972001 lambda_3_40 + 745708.3708891999 lambda_3_41
   + 473772.6683119999 lambda_0_42 + 759659.2217548001 lambda_3_43
   + 838101.2131668001 lambda_3_44 + 819248.407858 lambda_0_45
   + 816246.6332820002 lambda_1_46 + 665630.3132620003 lambda_2_47
   + 882802.4371039998 lambda_3_48 + 819248.407858 lambda_0_49
   + 816246.6332820002 lambda_1_50 + 665630.3132620003 lambda_2_51
   + 882802.4371039998 lambda_3_52 + 819248.407858 lambda_0_53
   + 816246.6332820002 lambda_1_54 + 665630.3132620003 lambda_2_55
   + 882802.4371039998 lambda_3_56 + 868780.8543635998 lambda_3_57
   + 819248.407858 lambda_0_58 + 816246.6332820002 lambda_1_59
   + 665630.3132620003 lambda_2_60 + 882802.4371039998 lambda_3_61
   + 512758.7532536 lambda_0_62 + 499074.3441491999 lambda_1_63
   + 626663.7677580002 lambda_2_64 + 487308.6587227999 lambda_3_65
   + 774526.0778236 lambda_0_66 + 774335.8961716 lambda_1_67
   + 473458.5784675999 lambda_3_68 + 690289.6444156 lambda_1_69
   + 819248.407858 lambda_0_70 + 816246.6332820002 lambda_1_71
   + 665630.3132620003 lambda_2_72 + 882802.4371039998 lambda_3_73
   + 556258.0093043998 lambda_3_74 + 544583.3040248 lambda_3_75
   + 594307.6084584 lambda_0_76 + 816246.6332820002 lambda_1_77
   + 665630.3132620003 lambda_2_78 + 882802.4371039998 lambda_3_79
   + 508777.4560104 lambda_0_80 + 485380.0117260001 lambda_0_81
   + 819248.407858 lambda_0_82 + 801819.0252104002 lambda_1_83
   + 665630.3132620003 lambda_2_84 + 842549.4658807999 lambda_3_85
   + 555448.1126371997 lambda_3_86
Subject To
 oneVisitCon0: lambda_0_0 + lambda_0_4 + lambda_0_8 + lambda_0_11
   + lambda_0_14 + lambda_0_17 + lambda_0_21 + lambda_0_24 + lambda_0_27
   + lambda_0_35 + lambda_0_39 + lambda_0_42 + lambda_0_45 + lambda_0_49
   + lambda_0_53 + lambda_0_58 + lambda_0_62 + lambda_0_66 + lambda_0_70
   + lambda_0_76 + lambda_0_80 + lambda_0_81 + lambda_0_82 = 1
 oneVisitCon1: lambda_1_1 + lambda_1_5 + lambda_1_9 + lambda_1_12
   + lambda_1_15 + lambda_1_18 + lambda_1_22 + lambda_1_25 + lambda_1_28
   + lambda_1_30 + lambda_1_32 + lambda_1_34 + lambda_1_36 + lambda_1_46
   + lambda_1_50 + lambda_1_54 + lambda_1_59 + lambda_1_63 + lambda_1_67
   + lambda_1_69 + lambda_1_71 + lambda_1_77 + lambda_1_83 = 1
 oneVisitCon2: lambda_2_2 + lambda_2_6 + lambda_2_19 + lambda_2_37
   + lambda_2_47 + lambda_2_51 + lambda_2_55 + lambda_2_60 + lambda_2_64
   + lambda_2_72 + lambda_2_78 + lambda_2_84 = 1
 oneVisitCon3: lambda_3_3 + lambda_3_7 + lambda_3_10 + lambda_3_13
   + lambda_3_16 + lambda_3_20 + lambda_3_23 + lambda_3_26 + lambda_3_29
   + lambda_3_31 + lambda_3_33 + lambda_3_38 + lambda_3_40 + lambda_3_41
   + lambda_3_43 + lambda_3_44 + lambda_3_48 + lambda_3_52 + lambda_3_56
   + lambda_3_57 + lambda_3_61 + lambda_3_65 + lambda_3_68 + lambda_3_73
   + lambda_3_74 + lambda_3_75 + lambda_3_79 + lambda_3_85 + lambda_3_86
   = 1
 visitedPickupCon0: lambda_0_24 + lambda_3_26 + lambda_0_62 + lambda_0_80
   + lambda_0_81 <= 1
 visitedPickupCon1: lambda_1_9 + lambda_1_12 + lambda_1_15 + lambda_1_18
   + lambda_3_20 + lambda_1_22 + lambda_1_30 + lambda_1_34 + lambda_1_46
   + lambda_1_50 + lambda_1_54 + lambda_1_59 + lambda_1_67 + lambda_1_69
   + lambda_1_71 + lambda_1_77 + lambda_1_83 <= 1
 visitedPickupCon2: lambda_1_5 + lambda_3_7 + lambda_0_8 + lambda_1_9
   + lambda_0_11 + lambda_1_12 + lambda_3_13 + lambda_0_21 + lambda_1_22
   + lambda_3_23 + lambda_1_30 + lambda_1_34 + lambda_0_35 + lambda_1_36
   + lambda_3_38 + lambda_1_46 + lambda_1_50 + lambda_1_54 + lambda_1_59
   + lambda_1_67 + lambda_1_71 + lambda_3_74 + lambda_0_76 + lambda_1_77
   + lambda_1_83 + lambda_3_85 <= 1
 visitedPickupCon3: lambda_0_8 + lambda_3_10 + lambda_0_11 + lambda_0_27
   + lambda_3_29 + lambda_0_35 + lambda_3_48 + lambda_3_52 + lambda_3_56
   + lambda_3_57 + lambda_3_61 + lambda_3_73 + lambda_0_76 + lambda_3_79
   + lambda_3_86 <= 1
 visitedPickupCon4: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_3_13
   + lambda_3_16 + lambda_1_36 + lambda_3_38 + lambda_3_40 + lambda_3_44
   + lambda_0_45 + lambda_0_49 + lambda_0_53 + lambda_0_58 + lambda_0_62
   + lambda_0_66 + lambda_0_70 + lambda_0_80 + lambda_0_81 + lambda_0_82
   <= 1
 visitedPickupCon5: lambda_2_6 + lambda_0_14 + lambda_2_37 + lambda_2_47
   + lambda_2_51 + lambda_2_55 + lambda_2_60 + lambda_2_64 + lambda_2_72
   + lambda_2_78 + lambda_2_84 <= 1
 visitedPickupCon6: lambda_2_6 + lambda_0_24 + lambda_3_26 + lambda_3_29
   + lambda_3_31 + lambda_2_37 + lambda_2_47 + lambda_2_51 + lambda_2_55
   + lambda_2_60 + lambda_2_72 + lambda_2_78 + lambda_0_80 + lambda_2_84
   <= 1
 visitedPickupCon7: lambda_1_12 + lambda_1_18 + lambda_3_20 + lambda_1_25
   + lambda_1_28 + lambda_1_32 + lambda_1_34 + lambda_0_62 + lambda_1_63
   + lambda_1_67 + lambda_1_69 + lambda_0_81 + lambda_1_83 <= 1
 visitedPickupCon8: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_1_9
   + lambda_1_12 + lambda_3_13 + lambda_1_15 + lambda_3_16 + lambda_1_30
   + lambda_1_36 + lambda_3_38 + lambda_3_40 + lambda_3_44 + lambda_0_45
   + lambda_1_46 + lambda_0_49 + lambda_1_50 + lambda_0_53 + lambda_1_54
   + lambda_0_58 + lambda_1_59 + lambda_0_62 + lambda_0_70 + lambda_1_71
   + lambda_1_77 + lambda_0_80 + lambda_0_81 + lambda_0_82 + lambda_1_83
   <= 1
 visitedPickupCon9: <= 1
 visitedPickupCon10: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_3_10
   + lambda_3_13 + lambda_3_16 + lambda_3_23 + lambda_3_33 + lambda_1_36
   + lambda_3_38 + lambda_3_40 + lambda_3_41 + lambda_3_43 + lambda_3_44
   + lambda_0_45 + lambda_3_48 + lambda_0_49 + lambda_3_52 + lambda_0_53
   + lambda_3_56 + lambda_3_57 + lambda_0_58 + lambda_3_61 + lambda_0_66
   + lambda_0_70 + lambda_3_73 + lambda_3_79 + lambda_0_82 + lambda_3_85
   <= 1
 visitedPickupCon11: lambda_1_9 + lambda_1_12 + lambda_1_15 + lambda_1_18
   + lambda_1_22 + lambda_0_24 + lambda_1_30 + lambda_1_34 + lambda_1_46
   + lambda_1_50 + lambda_1_54 + lambda_1_59 + lambda_1_67 + lambda_1_69
   + lambda_1_71 + lambda_1_77 + lambda_1_83 <= 1
 visitedPickupCon12: lambda_2_6 + lambda_0_14 + lambda_2_37 + lambda_2_47
   + lambda_2_51 + lambda_2_55 + lambda_2_60 + lambda_2_64 + lambda_2_72
   + lambda_2_78 + lambda_2_84 <= 1
 visitedPickupCon13: lambda_3_7 + lambda_3_10 + lambda_0_11 + lambda_3_16
   + lambda_0_21 + lambda_3_23 + lambda_1_28 + lambda_3_38 + lambda_0_42
   + lambda_3_43 + lambda_3_44 + lambda_3_48 + lambda_3_52 + lambda_3_56
   + lambda_3_61 + lambda_0_66 + lambda_3_68 + lambda_3_73 + lambda_3_74
   + lambda_0_76 + lambda_3_79 + lambda_3_85 <= 1
 visitedPickupCon14: lambda_0_8 + lambda_3_10 + lambda_0_11 + lambda_0_17
   + lambda_2_19 + lambda_0_27 + lambda_3_29 + lambda_3_31 + lambda_3_33
   + lambda_0_35 + lambda_3_48 + lambda_3_52 + lambda_3_56 + lambda_3_57
   + lambda_3_61 + lambda_3_73 + lambda_3_75 + lambda_0_76 + lambda_3_79
   + lambda_3_86 <= 1
 visitedPickupCon15: lambda_2_6 + lambda_2_37 + lambda_2_47 + lambda_2_51
   + lambda_2_55 + lambda_2_60 + lambda_2_64 + lambda_2_72 + lambda_2_78
   + lambda_2_84 <= 1
 visitedPickupCon16: lambda_1_25 + lambda_1_28 + lambda_1_32 <= 1
 visitedPickupCon17: lambda_0_21 + lambda_3_23 + lambda_0_24 + lambda_1_25
   + lambda_3_26 + lambda_1_28 + lambda_1_32 + lambda_0_39 + lambda_3_41
   + lambda_0_42 + lambda_3_43 + lambda_1_63 + lambda_3_65 + lambda_3_68
   + lambda_3_74 + lambda_3_85 <= 1
 visitedPickupCon18: lambda_0_14 + lambda_0_17 + lambda_0_21 + lambda_0_27
   + lambda_3_29 + lambda_3_31 + lambda_0_39 + lambda_0_42 + lambda_1_63
   + lambda_3_65 + lambda_3_68 + lambda_3_74 + lambda_3_75 + lambda_3_86
   <= 1
 visitedPickupCon19: lambda_0_4 + lambda_1_5 + lambda_3_7 + lambda_0_8
   + lambda_1_9 + lambda_3_10 + lambda_0_24 + lambda_1_25 + lambda_3_26
   + lambda_3_33 + lambda_0_35 + lambda_1_36 + lambda_3_38 + lambda_3_40
   + lambda_3_44 + lambda_0_45 + lambda_1_46 + lambda_3_48 + lambda_0_49
   + lambda_1_50 + lambda_3_52 + lambda_0_53 + lambda_1_54 + lambda_3_56
   + lambda_3_57 + lambda_0_58 + lambda_1_59 + lambda_3_61 + lambda_0_62
   + lambda_1_63 + lambda_3_65 + lambda_0_66 + lambda_1_67 + lambda_1_69
   + lambda_0_70 + lambda_1_71 + lambda_3_73 + lambda_1_77 + lambda_3_79
   + lambda_0_82 <= 1
Bounds
 lambda_0_0 <= 1
 lambda_1_1 <= 1
 lambda_2_2 <= 1
 lambda_3_3 <= 1
 lambda_0_4 <= 1
 lambda_1_5 = 0
 lambda_2_6 <= 1
 lambda_3_7 = 0
 lambda_0_8 <= 1
 lambda_1_9 = 0
 lambda_3_10 = 0
 lambda_0_11 <= 1
 lambda_1_12 <= 1
 lambda_3_13 = 0
 lambda_0_14 <= 1
 lambda_1_15 <= 1
 lambda_3_16 = 0
 lambda_0_17 <= 1
 lambda_1_18 <= 1
 lambda_2_19 <= 1
 lambda_3_20 <= 1
 lambda_0_21 = 0
 lambda_1_22 <= 1
 lambda_3_23 <= 1
 lambda_0_24 <= 1
 lambda_1_25 = 0
 lambda_3_26 = 0
 lambda_0_27 <= 1
 lambda_1_28 <= 1
 lambda_3_29 <= 1
 lambda_1_30 <= 1
 lambda_3_31 <= 1
 lambda_1_32 <= 1
 lambda_3_33 = 0
 lambda_1_34 <= 1
 lambda_0_35 <= 1
 lambda_1_36 = 0
 lambda_2_37 <= 1
 lambda_3_38 = 0
 lambda_0_39 = 0
 lambda_3_40 = 0
 lambda_3_41 <= 1
 lambda_0_42 = 0
 lambda_3_43 <= 1
 lambda_3_44 = 0
 lambda_0_45 <= 1
 lambda_1_46 = 0
 lambda_2_47 <= 1
 lambda_3_48 = 0
 lambda_0_49 <= 1
 lambda_1_50 = 0
 lambda_2_51 <= 1
 lambda_3_52 = 0
 lambda_0_53 <= 1
 lambda_1_54 = 0
 lambda_2_55 <= 1
 lambda_3_56 = 0
 lambda_3_57 = 0
 lambda_0_58 <= 1
 lambda_1_59 = 0
 lambda_2_60 <= 1
 lambda_3_61 = 0
 lambda_0_62 <= 1
 lambda_1_63 = 0
 lambda_2_64 <= 1
 lambda_3_65 = 0
 lambda_0_66 <= 1
 lambda_1_67 = 0
 lambda_3_68 <= 1
 lambda_1_69 = 0
 lambda_0_70 <= 1
 lambda_1_71 = 0
 lambda_2_72 <= 1
 lambda_3_73 = 0
 lambda_3_74 <= 1
 lambda_3_75 <= 1
 lambda_0_76 <= 1
 lambda_1_77 = 0
 lambda_2_78 <= 1
 lambda_3_79 = 0
 lambda_0_80 <= 1
 lambda_0_81 <= 1
End
