This is a implementation of column-and-constraint generation (C&CG) method solving a simple two stage robust optimization from the article published by Zeng & Zhao.

reference:Zeng, B., & Zhao, L. (2013). Solving two-stage robust optimization problems using a column-and-constraint generation method. Operations Research Letters, 41(5), 457–461. https://doi.org/10.1016/j.orl.2013.05.003
Average performance over 5 instance of C&CG algorithms on 10x10 instances with different Gamma.
 Gamma | Avg_Iterations | Avg_RunTime (seconds) | Successful_Runs |
|-----------|----------------|-----------------------|-----------------|
| 0.1       | 1.8            | 0.6769                | 5               |
| 0.2       | 2.6            | 0.7526                | 5               |
| 0.3       | 2.0            | 0.8161                | 5               |
| 0.4       | 3.0            | 0.9750                | 5               |
| 0.5       | 2.8            | 1.1089                | 5               |
| 0.6       | 4.8            | 1.4340                | 5               |
| 0.7       | 4.4            | 1.5898                | 5               |
| 0.8       | 2.8            | 1.0740                | 5               |
| 0.9       | 1.8            | 0.8585                | 5               |
| 1.0       | 1.0            | 1.4774                | 5               |


