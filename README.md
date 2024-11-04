This is a implementation of column-and-constraint generation (C&CG) method solving a simple two stage robust optimization from the article published by Zeng & Zhao.

reference:Zeng, B., & Zhao, L. (2013). Solving two-stage robust optimization problems using a column-and-constraint generation method. Operations Research Letters, 41(5), 457–461. https://doi.org/10.1016/j.orl.2013.05.003

Average performance over 10 instance of C&CG algorithms on 15x15 instances with different Gamma.
| Metric                  | 0.1   | 0.2   | 0.3   | 0.4   | 0.5   | 0.6   | 0.7   | 0.8   | 0.9   | 1.0   |
|-------------------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
| Benders Avg Runtime (sec)   | 3.0319 | 3.0821 | 3.8706 | 4.7504 | 2.6728 | 3.9569 | 3.4722 | 3.3910 | 3.0107 | 3.0599 |
| CCG Avg Runtime (sec)       | 2.0255 | 2.0587 | 3.5945 | 3.5547 | 3.6030 | 4.1741 | 4.7489 | 3.8669 | 3.9322 | 12.8903 |
| Runtime Ratio               | 1.50   | 1.50   | 1.08   | 1.34   | 0.74   | 0.95   | 0.73   | 0.88   | 0.77   | 0.24   |
| Benders Avg Iterations      | 17.9   | 19.1   | 23.0   | 26.0   | 16.4   | 23.9   | 20.5   | 21.0   | 18.3   | 18.1   |
| CCG Avg Iterations          | 3.3    | 3.3    | 5.1    | 4.6    | 4.3    | 5.0    | 5.0    | 3.9    | 3.8    | 2.0    |
| Iterations Ratio            | 5.42   | 5.79   | 4.51   | 5.65   | 3.81   | 4.78   | 4.10   | 5.38   | 4.82   | 9.05   |
