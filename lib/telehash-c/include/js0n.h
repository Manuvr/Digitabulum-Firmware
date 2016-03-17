
// key = string to match or null
// klen = key length (or 0), or if null key then len is the array offset value
// json = json object or array
// jlen = length of json
// vlen = where to store return value length
// returns pointer to value and sets len to value length, or 0 if not found or any error
char *js0n(char *key, size_t klen, char *json, size_t jlen, size_t *vlen);
