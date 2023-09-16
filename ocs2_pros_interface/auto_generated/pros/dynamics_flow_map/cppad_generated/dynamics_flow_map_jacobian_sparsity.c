void dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                         unsigned long const** col,
                                         unsigned long* nnz) {
   static unsigned long const rows[3] = {6,7,8};
   static unsigned long const cols[3] = {10,11,12};
   *row = rows;
   *col = cols;
   *nnz = 3;
}
