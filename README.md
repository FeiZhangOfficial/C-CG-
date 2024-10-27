This is a implementation of column-and-constraint generation (C&CG) method solving a simple two stage robust optimization from the article published by Zeng & Zhao.

reference:Zeng, B., & Zhao, L. (2013). Solving two-stage robust optimization problems using a column-and-constraint generation method. Operations Research Letters, 41(5), 457–461. https://doi.org/10.1016/j.orl.2013.05.003

Average performance over 5 instance of C&CG algorithms on 10x10 instances with different Gamma.
| Gamma | Avg_Iterations | Avg_RunTime (seconds) | Successful_Runs |
|-----------|----------------|----------------------|-----------------|
| 0.1       | 2.8            | 0.7263               | 5               |
| 0.2       | 4.6            | 0.9789               | 5               |
| 0.3       | 3.8            | 0.9108               | 5               |
| 0.4       | 4.2            | 1.3142               | 5               |
| 0.5       | 4.6            | 1.7823               | 5               |
| 0.6       | 3.4            | 1.4439               | 5               |
| 0.7       | 6.0            | 1.7759               | 5               |
| 0.8       | 3.4            | 1.0068               | 5               |
| 0.9       | 2.4            | 0.7760               | 5               |
| 1.0       | 2.0            | 1.2333               | 5               |


