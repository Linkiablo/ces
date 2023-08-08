typedef unsigned char u8;

#define WIDTH 3

static const u8 B[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
static const u8 B_t_expected[] = {0, 3, 6, 1, 4, 7, 2, 5, 8};

void transpose_b(u8 *B_t) {
    u8 i = 0;
    u8 j = 0;
    while (i < WIDTH) {
        j = 0;
        while (j < WIDTH) {
            B_t[j * WIDTH + i] = B[i * WIDTH + j];
            j++;
        }
        i++;
    }
}

int main() {
    u8 B_t[9];
    u8 i = 0;

    transpose_b(B_t);

    while (i < WIDTH * WIDTH) {
        if (B_t[i] != B_t_expected[i])
            return 1;

        i++;
    }

    return 0;
}
