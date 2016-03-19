#ifndef lob_h
#define lob_h

#include <stdint.h>
#include <stdlib.h>

typedef struct lob_struct
{
  // these are public but managed by accessors
  uint8_t *raw;
  uint8_t *body;
  size_t body_len;
  uint8_t *head;
  size_t head_len;
  
  // these are all for external use only
  uint32_t id;
  void *arg;

  // these are internal/private
  struct lob_struct *chain;
  char *cache; // edited copy of the json head

  // used only by the list utils
  struct lob_struct *next, *prev;

} *lob_t;

// these all allocate/free memory
lob_t lob_new();
lob_t lob_copy(lob_t p);
lob_t lob_free(lob_t p); // returns NULL for convenience

// creates a new parent packet chained to the given child one, so freeing the new packet also free's it
lob_t lob_chain(lob_t child);
// manually chain together two packets, returns parent, frees any existing child, creates parent if none
lob_t lob_link(lob_t parent, lob_t child);
// return a linked child if any
lob_t lob_linked(lob_t parent);
// returns child, unlinked
lob_t lob_unlink(lob_t parent);

// initialize head/body from raw, parses json
lob_t lob_parse(const uint8_t *raw, size_t len);

// return full encoded packet
uint8_t *lob_raw(lob_t p);
size_t lob_len(lob_t p);

// return null-terminated json header only
char *lob_json(lob_t p);

// set/store these in the current packet
uint8_t *lob_head(lob_t p, uint8_t *head, size_t len);
uint8_t *lob_body(lob_t p, uint8_t *body, size_t len);
lob_t lob_append(lob_t p, uint8_t *chunk, size_t len);
lob_t lob_append_str(lob_t p, char *chunk);

// convenient json setters/getters, always return given lob so they're chainable
lob_t lob_set_raw(lob_t p, char *key, size_t klen, char *val, size_t vlen); // raw
lob_t lob_set(lob_t p, char *key, char *val); // escapes value
lob_t lob_set_len(lob_t p, char *key, size_t klen, char *val, size_t vlen); // same as lob_set
lob_t lob_set_int(lob_t p, char *key, int val);
lob_t lob_set_uint(lob_t p, char *key, unsigned int val);
lob_t lob_set_float(lob_t p, char *key, float val, uint8_t places);
lob_t lob_set_printf(lob_t p, char *key, const char *format, ...);
lob_t lob_set_base32(lob_t p, char *key, uint8_t *val, size_t vlen);

// copies keys from json into p
lob_t lob_set_json(lob_t p, lob_t json);

// count of keys
unsigned int lob_keys(lob_t p);

// alpha-sorts the json keys
lob_t lob_sort(lob_t p);

// 0 to match, !0 if different, compares only top-level json and body
int lob_cmp(lob_t a, lob_t b);

// the return uint8_t* is invalidated with any _set* operation!
char *lob_get(lob_t p, char *key);
int lob_get_int(lob_t p, char *key);
unsigned int lob_get_uint(lob_t p, char *key);
float lob_get_float(lob_t p, char *key);

char *lob_get_index(lob_t p, uint32_t i); // returns ["0","1","2","3"] or {"0":"1","2":"3"}

// just shorthand for util_cmp to match a key/value
int lob_get_cmp(lob_t p, char *key, char *val);

// get the raw value, must use get_len
char *lob_get_raw(lob_t p, char *key);
size_t lob_get_len(lob_t p, char *key);

// returns new packets based on values
lob_t lob_get_json(lob_t p, char *key); // creates new packet from key:object value
lob_t lob_get_array(lob_t p, char *key); // list of packet->next from key:[object,object]
lob_t lob_get_base32(lob_t p, char *key); // decoded binary is the return body

// handles cloaking conveniently, len is lob_len()+(8*rounds)
uint8_t *lob_cloak(lob_t p, uint8_t rounds);

// decloaks and parses
lob_t lob_decloak(uint8_t *cloaked, size_t len);

// TODO, this would be handy, js syntax to get a json value
// char *lob_eval(lob_t p, "foo.bar[0]['zzz']");

// manage a basic double-linked list of packets using ->next and ->prev
lob_t lob_pop(lob_t list); // returns last item, item->next is the new list
lob_t lob_push(lob_t list, lob_t append); // appends new item, returns new list
lob_t lob_shift(lob_t list); // returns first item, item->next is the new list
lob_t lob_unshift(lob_t list, lob_t prepend); // adds item, returns new list
lob_t lob_splice(lob_t list, lob_t extract); // removes item from list, returns new list
lob_t lob_insert(lob_t list, lob_t after, lob_t p); // inserts item in list after other item, returns new list
lob_t lob_freeall(lob_t list); // frees all
lob_t lob_match(lob_t list, char *key, char *value); // find the first packet in the list w/ the matching key/value
lob_t lob_next(lob_t list);
lob_t lob_array(lob_t list); // return json array of the list

#endif
#include <stddef.h>

#ifndef xht_h
#define xht_h

// simple string->void* hashtable, very static and bare minimal, but efficient

typedef struct xht_struct *xht_t;

// must pass a prime#
xht_t xht_new(unsigned int prime);

// caller responsible for key storage, no copies made (don't free it b4 xht_free()!)
// set val to NULL to clear an entry, memory is reused but never free'd (# of keys only grows to peak usage)
void xht_set(xht_t h, const char *key, void *val);

// ooh! unlike set where key/val is in caller's mem, here they are copied into xht_t and free'd when val is 0 or xht_free()
void xht_store(xht_t h, const char *key, void *val, size_t vlen);

// returns value of val if found, or NULL
void *xht_get(xht_t h, const char *key);

// free the hashtable and all entries
void xht_free(xht_t h);

// pass a function that is called for every key that has a value set
typedef void (*xht_walker)(xht_t h, const char *key, void *val, void *arg);
void xht_walk(xht_t h, xht_walker w, void *arg);

// iterator through all the keys (NULL to start), use get for values
char *xht_iter(xht_t h, char *key);

#endif

#ifndef e3x_cipher_h
#define e3x_cipher_h

#include "lob.h"

// these are unique to each cipher set implementation
#define local_t void*
#define remote_t void*
#define ephemeral_t void*

// this is the overall holder for each cipher set, function pointers to cs specific implementations
typedef struct e3x_cipher_struct
{
  uint8_t id, csid;
  char hex[3], *alg;

  // these are common functions each one needs to support
  uint8_t *(*rand)(uint8_t *bytes, size_t len); // write len random bytes, returns bytes as well for convenience
  uint8_t *(*hash)(uint8_t *in, size_t len, uint8_t *out32); // sha256's the in, out32 must be [32] from caller
  uint8_t *(*err)(void); // last known crypto error string, if any

  // create a new keypair, save encoded to csid in each
  uint8_t (*generate)(lob_t keys, lob_t secrets);

  // our local identity
  local_t (*local_new)(lob_t keys, lob_t secrets);
  void (*local_free)(local_t local);
  lob_t (*local_decrypt)(local_t local, lob_t outer);
  lob_t (*local_sign)(local_t local, lob_t args, uint8_t *data, size_t len);
  
  // a remote endpoint identity
  remote_t (*remote_new)(lob_t key, uint8_t *token);
  void (*remote_free)(remote_t remote);
  uint8_t (*remote_verify)(remote_t remote, local_t local, lob_t outer);
  lob_t (*remote_encrypt)(remote_t remote, local_t local, lob_t inner);
  uint8_t (*remote_validate)(remote_t remote, lob_t args, lob_t sig, uint8_t *data, size_t len);
  
  // an active session to a remote for channel packets
  ephemeral_t (*ephemeral_new)(remote_t remote, lob_t outer);
  void (*ephemeral_free)(ephemeral_t ephemeral);
  lob_t (*ephemeral_encrypt)(ephemeral_t ephemeral, lob_t inner);
  lob_t (*ephemeral_decrypt)(ephemeral_t ephemeral, lob_t outer);
} *e3x_cipher_t;


// all possible cipher sets, as index into cipher_sets global
#define CS_1a 0
#define CS_2a 1
#define CS_3a 2
#define CS_MAX 3

extern e3x_cipher_t e3x_cipher_sets[]; // all created
extern e3x_cipher_t e3x_cipher_default; // just one of them for the rand/hash utils

// calls all e3x_cipher_init_*'s to fill in e3x_cipher_sets[]
uint8_t e3x_cipher_init(lob_t options);

// return by id or hex
e3x_cipher_t e3x_cipher_set(uint8_t csid, char *hex);

// init functions for each
e3x_cipher_t cs1a_init(lob_t options);
e3x_cipher_t cs2a_init(lob_t options);
e3x_cipher_t cs3a_init(lob_t options);

#endif
#ifndef e3x_self_h
#define e3x_self_h

#include "e3x_cipher.h"

typedef struct e3x_self_struct
{
  lob_t keys[CS_MAX];
  local_t locals[CS_MAX];
} *e3x_self_t;

// load id secrets/keys to create a new local endpoint
e3x_self_t e3x_self_new(lob_t secrets, lob_t keys);
void e3x_self_free(e3x_self_t self); // any exchanges must have been free'd first

// try to decrypt any message sent to us, returns the inner
lob_t e3x_self_decrypt(e3x_self_t self, lob_t message);

// generate a signature for the data
lob_t e3x_self_sign(e3x_self_t self, lob_t args, uint8_t *data, size_t len);

#endif
#ifndef e3x_exchange_h
#define e3x_exchange_h

#include <stdint.h>
#include "e3x_cipher.h"
#include "e3x_self.h"

// apps should only use accessor functions for values in this struct
typedef struct e3x_exchange_struct
{
  e3x_cipher_t cs; // convenience
  e3x_self_t self;
  uint8_t csid, order;
  char hex[3];
  remote_t remote;
  ephemeral_t ephem;
  uint8_t token[16], eid[16];
  uint32_t in, out;
  uint32_t cid, last;
} *e3x_exchange_t;

// make a new exchange
// packet must contain the raw key in the body
e3x_exchange_t e3x_exchange_new(e3x_self_t self, uint8_t csid, lob_t key);
void e3x_exchange_free(e3x_exchange_t x);

// these are stateless async encryption and verification
lob_t e3x_exchange_message(e3x_exchange_t x, lob_t inner);
uint8_t e3x_exchange_verify(e3x_exchange_t x, lob_t outer);
uint8_t e3x_exchange_validate(e3x_exchange_t x, lob_t args, lob_t sig, uint8_t *data, size_t len);

// return the current incoming at value, optional arg to update it
uint32_t e3x_exchange_in(e3x_exchange_t x, uint32_t at);

// will return the current outgoing at value, optional arg to update it
uint32_t e3x_exchange_out(e3x_exchange_t x, uint32_t at);

// synchronize to incoming ephemeral key and set out at = in at, returns x if success, NULL if not
e3x_exchange_t e3x_exchange_sync(e3x_exchange_t x, lob_t outer);

// drops ephemeral state, out=0
e3x_exchange_t e3x_exchange_down(e3x_exchange_t x);

// generates handshake w/ current e3x_exchange_out value and ephemeral key
lob_t e3x_exchange_handshake(e3x_exchange_t x, lob_t inner);

// simple synchronous encrypt/decrypt conversion of any packet for channels
lob_t e3x_exchange_receive(e3x_exchange_t x, lob_t outer); // goes to channel, validates cid
lob_t e3x_exchange_send(e3x_exchange_t x, lob_t inner); // comes from channel 

// validate the next incoming channel id from the packet, or return the next avail outgoing channel id
uint32_t e3x_exchange_cid(e3x_exchange_t x, lob_t incoming);

// get the 16-byte token value to this exchange
uint8_t *e3x_exchange_token(e3x_exchange_t x);

#endif
#ifndef hashname_h
#define hashname_h

#include "base32.h"
#include "lob.h"

// overall type
typedef struct hashname_struct *hashname_t;

// only things that actually malloc/free
hashname_t hashname_dup(hashname_t hn);
hashname_t hashname_free(hashname_t hn);

// everything else returns a pointer to a static global for temporary use
hashname_t hashname_vchar(const char *str); // from a string
hashname_t hashname_vbin(const uint8_t *bin);
hashname_t hashname_vkeys(lob_t keys);
hashname_t hashname_vkey(lob_t key, uint8_t id); // key is body, intermediates in json

// accessors
uint8_t *hashname_bin(hashname_t hn); // 32 bytes
char *hashname_char(hashname_t hn); // 52 byte base32 string w/ \0 (TEMPORARY)
char *hashname_short(hashname_t hn); // 16 byte base32 string w/ \0 (TEMPORARY)

// utilities related to hashnames
int hashname_cmp(hashname_t a, hashname_t b);  // memcmp shortcut
uint8_t hashname_id(lob_t a, lob_t b); // best matching id (single byte)
lob_t hashname_im(lob_t keys, uint8_t id); // intermediate hashes in the json, optional id to set that as body


#endif
#ifndef mesh_h
#define mesh_h

typedef struct mesh_struct *mesh_t;
typedef struct link_struct *link_t;
typedef struct pipe_struct *pipe_t;
typedef struct chan_struct *chan_t;



#include "e3x.h"
#include "lib.h"
#include "util.h"
#include "chan.h"
#include "pipe.h"
#include "link.h"

struct mesh_struct
{
  hashname_t id;
  char *uri;
  lob_t keys, paths;
  e3x_self_t self;
  xht_t index; // for extensions to use
  void *on; // internal list of triggers
  // shared network info
  uint16_t port_local, port_public;
  char *ipv4_local, *ipv4_public;
  lob_t handshakes, cached; // handshakes
  void *routes; // internal routing
  link_t links;
};

// pass in a prime for the main index of hashnames+links+channels, 0 to use compiled default
mesh_t mesh_new(uint32_t prime);
mesh_t mesh_free(mesh_t mesh);

// must be called to initialize to a hashname from keys/secrets, return !0 if failed
uint8_t mesh_load(mesh_t mesh, lob_t secrets, lob_t keys);

// creates and loads a new random hashname, returns secrets if it needs to be saved/reused
lob_t mesh_generate(mesh_t mesh);

// return the best current URI to this endpoint, optional base
char *mesh_uri(mesh_t mesh, char *base);

// generate json of mesh keys and current paths
lob_t mesh_json(mesh_t mesh);

// generate json for all links, returns lob list
lob_t mesh_links(mesh_t mesh);

// creates a link from the json format of {"hashname":"...","keys":{},"paths":[]}, optional direct pipe too
link_t mesh_add(mesh_t mesh, lob_t json, pipe_t pipe);

// return only if this hashname (full or short) is currently linked (in any state)
link_t mesh_linked(mesh_t mesh, char *hn, size_t len);
link_t mesh_linkid(mesh_t mesh, hashname_t id); // TODO, clean this up

// remove this link, will event it down and clean up during next process()
mesh_t mesh_unlink(link_t link);

// add a custom outgoing handshake packet to all links
mesh_t mesh_handshake(mesh_t mesh, lob_t handshake);

// query the cache of handshakes for a matching one with a specific type
lob_t mesh_handshakes(mesh_t mesh, lob_t handshake, char *type);

// processes incoming packet, it will take ownership of packet, returns link delivered to if success
link_t mesh_receive(mesh_t mesh, lob_t packet, pipe_t pipe);

// process any unencrypted handshake packet, cache if needed
link_t mesh_receive_handshake(mesh_t mesh, lob_t handshake, pipe_t pipe);

// process any channel timeouts based on the current/given time
mesh_t mesh_process(mesh_t mesh, uint32_t now);

// adds a forwarding route for any incoming packet w/ this token
mesh_t mesh_forward(mesh_t m, uint8_t *token, link_t to, uint8_t flag);

// callback when the mesh is free'd
void mesh_on_free(mesh_t mesh, char *id, void (*free)(mesh_t mesh));

// callback when a path needs to be turned into a pipe
void mesh_on_path(mesh_t mesh, char *id, pipe_t (*path)(link_t link, lob_t path));
pipe_t mesh_path(mesh_t mesh, link_t link, lob_t path);

// callback when an unknown hashname is discovered
void mesh_on_discover(mesh_t mesh, char *id, link_t (*discover)(mesh_t mesh, lob_t discovered, pipe_t pipe));
void mesh_discover(mesh_t mesh, lob_t discovered, pipe_t pipe);

// callback when a link changes state created/up/down
void mesh_on_link(mesh_t mesh, char *id, void (*link)(link_t link));
void mesh_link(mesh_t mesh, link_t link);

// callback when a new incoming channel is requested
void mesh_on_open(mesh_t mesh, char *id, lob_t (*open)(link_t link, lob_t open));
lob_t mesh_open(mesh_t mesh, link_t link, lob_t open);


#endif
#ifndef link_h
#define link_h
#include <stdint.h>

#include "mesh.h"

struct link_struct
{
  char handle[17]; // b32 hashname_short
  uint8_t csid;

  // public link data
  hashname_t id;
  e3x_exchange_t x;
  mesh_t mesh;
  lob_t key;
  lob_t handshakes;
  chan_t chans;
  
  // these are for internal link management only
  struct seen_struct *pipes;
  link_t next;
};

// these all create or return existing one from the mesh
link_t link_get(mesh_t mesh, hashname_t id);
link_t link_keys(mesh_t mesh, lob_t keys); // adds in the right key
link_t link_key(mesh_t mesh, lob_t key, uint8_t csid); // adds in from the body

// get link info json
lob_t link_json(link_t link);

// removes from mesh
void link_free(link_t link);

// load in the key to existing link
link_t link_load(link_t link, uint8_t csid, lob_t key);

// try to turn a path into a pipe and add it to the link
pipe_t link_path(link_t link, lob_t path);

// just manage a pipe directly, removes if pipe->down, else adds
link_t link_pipe(link_t link, pipe_t pipe);

// iterate through existing pipes for a link
pipe_t link_pipes(link_t link, pipe_t after);

// add a custom outgoing handshake packet for this link
link_t link_handshake(link_t link, lob_t handshake);

// process a decrypted channel packet
link_t link_receive(link_t link, lob_t inner, pipe_t pipe);

// process an incoming handshake
link_t link_receive_handshake(link_t link, lob_t handshake, pipe_t pipe);

// try to deliver this packet to the best pipe
link_t link_send(link_t link, lob_t inner);

// return current handshake(s)
lob_t link_handshakes(link_t link);

// send current handshake(s) to all pipes and return them
lob_t link_sync(link_t link);

// generate new encrypted handshake(s) and sync
lob_t link_resync(link_t link);

// is the other endpoint connected and the link available, NULL if not
link_t link_up(link_t link);

// force link down, ends channels and generates events
link_t link_down(link_t link);

// create/track a new channel for this open
chan_t link_chan(link_t link, lob_t open);

// encrypt and send this one packet on this pipe
link_t link_direct(link_t link, lob_t inner, pipe_t pipe);

// process any channel timeouts based on the current/given time
link_t link_process(link_t link, uint32_t now);

#endif
#ifndef chan_h
#define chan_h
#include <stdint.h>
#include "mesh.h"

enum chan_states { CHAN_ENDED, CHAN_OPENING, CHAN_OPEN };

// standalone channel packet management, buffering and ordering
// internal only structure, always use accessors
struct chan_struct
{
  link_t link; // so channels can be first-class
  chan_t next; // links keep lists
  uint32_t id; // wire id (not unique)
  char *type;
  lob_t open; // cached for convenience
  uint32_t capacity, max; // totals for windowing

  // timer stuff
  uint32_t tsent, trecv; // last send, recv at
  uint32_t timeout; // when in the future to trigger timeout
  
  // reliable tracking
  lob_t sent;
  lob_t in;
  uint32_t seq, ack, acked, window;
  
  // direct handler
  void *arg;
  void (*handle)(chan_t c, void *arg);

  enum chan_states state;
};

// caller must manage lists of channels per e3x_exchange based on cid
chan_t chan_new(lob_t open); // open must be chan_receive or chan_send next yet
chan_t chan_free(chan_t c);

// sets when in the future this channel should timeout auto-error from no receive, returns current timeout
uint32_t chan_timeout(chan_t c, uint32_t at);

// sets the max size (in bytes) of all buffered data in or out, returns current usage, can pass 0 just to check
uint32_t chan_size(chan_t c, uint32_t max); // will actively signal incoming window size depending on capacity left

// incoming packets
chan_t chan_receive(chan_t c, lob_t inner); // process into receiving queue
chan_t chan_sync(chan_t c, uint8_t sync); // false to force start timeouts (after any new handshake), true to cancel and resend last packet (after any e3x_exchange_sync)
lob_t chan_receiving(chan_t c); // get next avail packet in order, null if nothing

// outgoing packets
lob_t chan_oob(chan_t c); // id/ack/miss only headers base packet
lob_t chan_packet(chan_t c);  // creates a sequenced packet w/ all necessary headers, just a convenience
chan_t chan_send(chan_t c, lob_t inner); // encrypts and sends packet out link
chan_t chan_err(chan_t c, char *err); // generates local-only error packet for next chan_process()

// must be called after every send or receive, processes resends/timeouts, fires handlers
chan_t chan_process(chan_t c, uint32_t now);

// set up internal handler for all incoming packets on this channel
chan_t chan_handle(chan_t c, void (*handle)(chan_t c, void *arg), void *arg);

// convenience functions, accessors
chan_t chan_next(chan_t c); // c->next
uint32_t chan_id(chan_t c); // c->id
lob_t chan_open(chan_t c); // c->open
enum chan_states chan_state(chan_t c);



#endif
#ifndef util_chunks_h
#define util_chunks_h

#include <stdint.h>
#include "lob.h"

// for list of incoming chunks
typedef struct util_chunk_struct
{
  struct util_chunk_struct *prev;
  uint8_t size;
  uint8_t data[];
} *util_chunk_t;

typedef struct util_chunks_struct
{

  util_chunk_t reading; // stacked linked list of incoming chunks

  lob_t writing;
  size_t writeat; // offset into lob_raw()
  uint16_t waitat; // gets to 256, offset into current chunk
  uint8_t waiting; // current writing chunk size;
  uint8_t readat; // always less than a max chunk, offset into reading

  uint8_t cap;
  uint8_t blocked:1, blocking:1, ack:1, err:1; // bool flags
} *util_chunks_t;


// size of each chunk, 0 == MAX (256)
util_chunks_t util_chunks_new(uint8_t size);

util_chunks_t util_chunks_free(util_chunks_t chunks);

// turn this packet into chunks and append, free's out
util_chunks_t util_chunks_send(util_chunks_t chunks, lob_t out);

// get any packets that have been reassembled from incoming chunks
lob_t util_chunks_receive(util_chunks_t chunks);

// bytes waiting to be sent
uint32_t util_chunks_writing(util_chunks_t chunks);


////// these are for a stream-based transport

// how many bytes are there ready to write
uint32_t util_chunks_len(util_chunks_t chunks);

// return the next block of data to be written to the stream transport, max len is util_chunks_len()
uint8_t *util_chunks_write(util_chunks_t chunks);

// advance the write this far, don't mix with util_chunks_out() usage
util_chunks_t util_chunks_written(util_chunks_t chunks, size_t len);

// queues incoming stream based data
util_chunks_t util_chunks_read(util_chunks_t chunks, uint8_t *block, size_t len);

////// these are for frame-based transport

// size of the next chunk, -1 when none, max is chunks size-1
int16_t util_chunks_size(util_chunks_t chunks);

// return the next chunk of data, use util_chunks_next to advance
uint8_t *util_chunks_frame(util_chunks_t chunks);

// peek into what the next chunk size will be, to see terminator ones
int16_t util_chunks_peek(util_chunks_t chunks);

// process incoming chunk
util_chunks_t util_chunks_chunk(util_chunks_t chunks, uint8_t *chunk, int16_t size);

// advance the write past the current chunk
util_chunks_t util_chunks_next(util_chunks_t chunks);


#endif
#ifndef util_frames_h
#define util_frames_h

#include <stdint.h>
#include "lob.h"

// for list of incoming frames
typedef struct util_frame_struct
{
  struct util_frame_struct *prev;
  uint8_t data[];
} *util_frame_t;

typedef struct util_frames_struct
{

  lob_t inbox; // received packets waiting to be processed
  lob_t outbox; // current packet being sent out

  util_frame_t cache; // stacked linked list of incoming frames in progress

  uint32_t inlast; // last good incoming frame hash
  uint32_t outbase; // last sent outbox frame hash

  uint8_t in; // number of incoming frames received/waiting
  uint8_t out; //  number of outgoing frames of outbox sent since outbase

  uint8_t size; // frame size
  uint8_t flush:1; // bool to signal a flush is needed
  uint8_t err:1; // unrecoverable failure

} *util_frames_t;


// size of each frame, min 16 max 128, multiple of 4
util_frames_t util_frames_new(uint8_t size);

util_frames_t util_frames_free(util_frames_t frames);

// turn this packet into frames and append, free's out
util_frames_t util_frames_send(util_frames_t frames, lob_t out);

// get any packets that have been reassembled from incoming frames
lob_t util_frames_receive(util_frames_t frames);

// total bytes in the inbox/outbox
size_t util_frames_inlen(util_frames_t frames);
size_t util_frames_outlen(util_frames_t frames);

// the next frame of data in/out, if data NULL return is just ready check bool
util_frames_t util_frames_inbox(util_frames_t frames, uint8_t *data);
util_frames_t util_frames_outbox(util_frames_t frames, uint8_t *data);

// is there an expectation of an incoming frame
util_frames_t util_frames_await(util_frames_t frames);

#endif
// Base32 implementation
//
// Copyright 2010 Google Inc.
// Author: Markus Gutschke
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Encode and decode from base32 encoding using the following alphabet:
//   ABCDEFGHIJKLMNOPQRSTUVWXYZ234567
// This alphabet is documented in RFC 4668/3548
//
// We allow white-space and hyphens, but all other characters are considered
// invalid.
//
// All functions return the number of output bytes or -1 on error. If the
// output buffer is too small, the result will silently be truncated.

#ifndef _BASE32_H_
#define _BASE32_H_

#include <stdint.h>
#include <stddef.h>

size_t base32_decode(const char *encoded, size_t length, uint8_t *result, size_t bufSize);
size_t base32_encode(const uint8_t *data, size_t length, char *result, size_t bufSize);

// number of characters required, including null terminator
size_t base32_encode_length(size_t rawLength);

// number of bytes, rounded down (truncates extra bits)
size_t base32_decode_floor(size_t base32Length);

#endif /* _BASE32_H_ */
#ifndef b64_h
#define b64_h

#include <stddef.h>
#include <stdint.h>

// length of data resulting from encoding/decoding
#define base64_encode_length(x) (8 * (((x) + 2) / 6)) + 1
#define base64_decode_length(x) ((((x) + 2) * 6) / 8)

// encode str of len into out (must be at least base64_encode_length(len) big), return encoded len
size_t base64_encoder(const uint8_t *str, size_t len, char *out);

// decode str of len into out (must be base64_decode_length(len) bit), return actual decoded len
size_t base64_decoder(const char *str, size_t len, uint8_t *out);

#endif

#ifndef _CHACHA20_H_
#define _CHACHA20_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// a convert-in-place utility
uint8_t *chacha20(uint8_t *key, uint8_t *nonce, uint8_t *bytes, uint32_t len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* !_CHACHA20_H_ */
#ifndef chan_h
#define chan_h
#include <stdint.h>
#include "mesh.h"

enum chan_states { CHAN_ENDED, CHAN_OPENING, CHAN_OPEN };

// standalone channel packet management, buffering and ordering
// internal only structure, always use accessors
struct chan_struct
{
  link_t link; // so channels can be first-class
  chan_t next; // links keep lists
  uint32_t id; // wire id (not unique)
  char *type;
  lob_t open; // cached for convenience
  uint32_t capacity, max; // totals for windowing

  // timer stuff
  uint32_t tsent, trecv; // last send, recv at
  uint32_t timeout; // when in the future to trigger timeout
  
  // reliable tracking
  lob_t sent;
  lob_t in;
  uint32_t seq, ack, acked, window;
  
  // direct handler
  void *arg;
  void (*handle)(chan_t c, void *arg);

  enum chan_states state;
};

// caller must manage lists of channels per e3x_exchange based on cid
chan_t chan_new(lob_t open); // open must be chan_receive or chan_send next yet
chan_t chan_free(chan_t c);

// sets when in the future this channel should timeout auto-error from no receive, returns current timeout
uint32_t chan_timeout(chan_t c, uint32_t at);

// sets the max size (in bytes) of all buffered data in or out, returns current usage, can pass 0 just to check
uint32_t chan_size(chan_t c, uint32_t max); // will actively signal incoming window size depending on capacity left

// incoming packets
chan_t chan_receive(chan_t c, lob_t inner); // process into receiving queue
chan_t chan_sync(chan_t c, uint8_t sync); // false to force start timeouts (after any new handshake), true to cancel and resend last packet (after any e3x_exchange_sync)
lob_t chan_receiving(chan_t c); // get next avail packet in order, null if nothing

// outgoing packets
lob_t chan_oob(chan_t c); // id/ack/miss only headers base packet
lob_t chan_packet(chan_t c);  // creates a sequenced packet w/ all necessary headers, just a convenience
chan_t chan_send(chan_t c, lob_t inner); // encrypts and sends packet out link
chan_t chan_err(chan_t c, char *err); // generates local-only error packet for next chan_process()

// must be called after every send or receive, processes resends/timeouts, fires handlers
chan_t chan_process(chan_t c, uint32_t now);

// set up internal handler for all incoming packets on this channel
chan_t chan_handle(chan_t c, void (*handle)(chan_t c, void *arg), void *arg);

// convenience functions, accessors
chan_t chan_next(chan_t c); // c->next
uint32_t chan_id(chan_t c); // c->id
lob_t chan_open(chan_t c); // c->open
enum chan_states chan_state(chan_t c);



#endif
#ifndef e3x_cipher_h
#define e3x_cipher_h

#include "lob.h"

// these are unique to each cipher set implementation
#define local_t void*
#define remote_t void*
#define ephemeral_t void*

// this is the overall holder for each cipher set, function pointers to cs specific implementations
typedef struct e3x_cipher_struct
{
  uint8_t id, csid;
  char hex[3], *alg;

  // these are common functions each one needs to support
  uint8_t *(*rand)(uint8_t *bytes, size_t len); // write len random bytes, returns bytes as well for convenience
  uint8_t *(*hash)(uint8_t *in, size_t len, uint8_t *out32); // sha256's the in, out32 must be [32] from caller
  uint8_t *(*err)(void); // last known crypto error string, if any

  // create a new keypair, save encoded to csid in each
  uint8_t (*generate)(lob_t keys, lob_t secrets);

  // our local identity
  local_t (*local_new)(lob_t keys, lob_t secrets);
  void (*local_free)(local_t local);
  lob_t (*local_decrypt)(local_t local, lob_t outer);
  lob_t (*local_sign)(local_t local, lob_t args, uint8_t *data, size_t len);
  
  // a remote endpoint identity
  remote_t (*remote_new)(lob_t key, uint8_t *token);
  void (*remote_free)(remote_t remote);
  uint8_t (*remote_verify)(remote_t remote, local_t local, lob_t outer);
  lob_t (*remote_encrypt)(remote_t remote, local_t local, lob_t inner);
  uint8_t (*remote_validate)(remote_t remote, lob_t args, lob_t sig, uint8_t *data, size_t len);
  
  // an active session to a remote for channel packets
  ephemeral_t (*ephemeral_new)(remote_t remote, lob_t outer);
  void (*ephemeral_free)(ephemeral_t ephemeral);
  lob_t (*ephemeral_encrypt)(ephemeral_t ephemeral, lob_t inner);
  lob_t (*ephemeral_decrypt)(ephemeral_t ephemeral, lob_t outer);
} *e3x_cipher_t;


// all possible cipher sets, as index into cipher_sets global
#define CS_1a 0
#define CS_2a 1
#define CS_3a 2
#define CS_MAX 3

extern e3x_cipher_t e3x_cipher_sets[]; // all created
extern e3x_cipher_t e3x_cipher_default; // just one of them for the rand/hash utils

// calls all e3x_cipher_init_*'s to fill in e3x_cipher_sets[]
uint8_t e3x_cipher_init(lob_t options);

// return by id or hex
e3x_cipher_t e3x_cipher_set(uint8_t csid, char *hex);

// init functions for each
e3x_cipher_t cs1a_init(lob_t options);
e3x_cipher_t cs2a_init(lob_t options);
e3x_cipher_t cs3a_init(lob_t options);

#endif
#ifndef e3x_exchange_h
#define e3x_exchange_h

#include <stdint.h>
#include "e3x_cipher.h"
#include "e3x_self.h"

// apps should only use accessor functions for values in this struct
typedef struct e3x_exchange_struct
{
  e3x_cipher_t cs; // convenience
  e3x_self_t self;
  uint8_t csid, order;
  char hex[3];
  remote_t remote;
  ephemeral_t ephem;
  uint8_t token[16], eid[16];
  uint32_t in, out;
  uint32_t cid, last;
} *e3x_exchange_t;

// make a new exchange
// packet must contain the raw key in the body
e3x_exchange_t e3x_exchange_new(e3x_self_t self, uint8_t csid, lob_t key);
void e3x_exchange_free(e3x_exchange_t x);

// these are stateless async encryption and verification
lob_t e3x_exchange_message(e3x_exchange_t x, lob_t inner);
uint8_t e3x_exchange_verify(e3x_exchange_t x, lob_t outer);
uint8_t e3x_exchange_validate(e3x_exchange_t x, lob_t args, lob_t sig, uint8_t *data, size_t len);

// return the current incoming at value, optional arg to update it
uint32_t e3x_exchange_in(e3x_exchange_t x, uint32_t at);

// will return the current outgoing at value, optional arg to update it
uint32_t e3x_exchange_out(e3x_exchange_t x, uint32_t at);

// synchronize to incoming ephemeral key and set out at = in at, returns x if success, NULL if not
e3x_exchange_t e3x_exchange_sync(e3x_exchange_t x, lob_t outer);

// drops ephemeral state, out=0
e3x_exchange_t e3x_exchange_down(e3x_exchange_t x);

// generates handshake w/ current e3x_exchange_out value and ephemeral key
lob_t e3x_exchange_handshake(e3x_exchange_t x, lob_t inner);

// simple synchronous encrypt/decrypt conversion of any packet for channels
lob_t e3x_exchange_receive(e3x_exchange_t x, lob_t outer); // goes to channel, validates cid
lob_t e3x_exchange_send(e3x_exchange_t x, lob_t inner); // comes from channel 

// validate the next incoming channel id from the packet, or return the next avail outgoing channel id
uint32_t e3x_exchange_cid(e3x_exchange_t x, lob_t incoming);

// get the 16-byte token value to this exchange
uint8_t *e3x_exchange_token(e3x_exchange_t x);

#endif
#ifndef e3x_h
#define e3x_h

#define E3X_VERSION_MAJOR 0
#define E3X_VERSION_MINOR 5
#define E3X_VERSION_PATCH 1
#define E3X_VERSION ((E3X_VERSION_MAJOR) * 10000 + (E3X_VERSION_MINOR) * 100 + (E3X_VERSION_PATCH))

#ifdef __cplusplus
extern "C" {
#endif

#include "lob.h" // json+binary container

// ##### e3x - end-to-end encrypted exchange #####
//
// * intended to be wrapped/embedded in other codebases
// * includes some minimal crypto, but is primarily a wrapper around other crypto libraries
// * tries to minimize the integration points to send and receive encrypted packets over any transport
// * everything contains a '3' to minimize any naming conflicts when used with other codebases
//


// top-level library functions

// process-wide boot one-time initialization, !0 is error and lob_get(options,"err");
uint8_t e3x_init(lob_t options);

// return last known error string from anywhere, intended only for debugging/logging
uint8_t *e3x_err(void);

// generate a new local identity, secrets returned in the lob json and keys in the linked lob json
lob_t e3x_generate(void);

// random bytes, from a supported cipher set
uint8_t *e3x_rand(uint8_t *bytes, size_t len);

// sha256 hashing, from one of the cipher sets
uint8_t *e3x_hash(uint8_t *in, size_t len, uint8_t *out32);


// local endpoint state management
#include "e3x_self.h"

// a single exchange (a session w/ local endpoint and remote endpoint)
#include "e3x_exchange.h"



//##############
/* binding notes

* app must have an index of the hashnames-to-exchange and tokens-to-exchange
* for each exchange, keep a list of active network transport sessions and potential transport paths
* also keep a list of active channels per exchange to track state
* one transport session may be delivering to multiple exchanges (not a 1:1 mapping)
* unreliable transport sessions must trigger a new handshake for any exchange when they need to be re-validated
* all transport sessions must signal when they are closed, which generates a new handshake to any other sessions
* always use the most recently validated-active transport session to deliver to for sending

*/

#ifdef __cplusplus
}
#endif

#endif
#ifndef e3x_self_h
#define e3x_self_h

#include "e3x_cipher.h"

typedef struct e3x_self_struct
{
  lob_t keys[CS_MAX];
  local_t locals[CS_MAX];
} *e3x_self_t;

// load id secrets/keys to create a new local endpoint
e3x_self_t e3x_self_new(lob_t secrets, lob_t keys);
void e3x_self_free(e3x_self_t self); // any exchanges must have been free'd first

// try to decrypt any message sent to us, returns the inner
lob_t e3x_self_decrypt(e3x_self_t self, lob_t message);

// generate a signature for the data
lob_t e3x_self_sign(e3x_self_t self, lob_t args, uint8_t *data, size_t len);

#endif
#ifndef ext_block_h
#define ext_block_h

#include "mesh.h"

// add block channel support
mesh_t ext_block(mesh_t mesh);

// get the next incoming block, if any, packet->arg is the link it came from
lob_t ext_block_receive(mesh_t mesh);

// creates/reuses a single default block channel on the link
link_t ext_block_send(link_t link, lob_t block);

// TODO, handle multiple block channels per link, and custom packets on open

#endif
#ifndef ext_chat_h
#define ext_chat_h

/*
#include "switch.h"

typedef struct chat_struct 
{
  char ep[32+1], id[32+1+64+1], idhash[9];
  hashname_t origin;
  switch_t s;
  chan_t hub;
  char rhash[9];
  uint8_t local, seed[4];
  uint16_t seq;
  lob_t roster;
  xht_t conn, log;
  lob_t msgs;
  char *join, *sent, *after;
} *chat_t;

chat_t ext_chat(chan_t c);

chat_t chat_get(switch_t s, char *id);
chat_t chat_free(chat_t chat);

// get the next incoming message (type state/message), caller must free
lob_t chat_pop(chat_t chat);

lob_t chat_message(chat_t chat);
chat_t chat_join(chat_t chat, lob_t join);
chat_t chat_send(chat_t chat, lob_t msg);
chat_t chat_add(chat_t chat, char *hn, char *val);

// get a participant or walk the list, returns the current state packet (immutable), online:true/false
lob_t chat_participant(chat_t chat, char *hn);
lob_t chat_iparticipant(chat_t chat, int index);

*/
#endif
#ifndef ext_connect_h
#define ext_connect_h

//#include "ext.h"

//void ext_connect(chan_t c);

#endif
#ifndef ext_h
#define ext_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mesh.h"

#include "ext_stream.h"
#include "ext_block.h"
#include "ext_path.h"
#include "ext_peer.h"

//#include "chat.h"
//#include "thtp.h"
//#include "connect.h"
//#include "sock.h"

#endif
#ifndef ext_path_h
#define ext_path_h

#include "ext.h"

// send a path ping and get callback event
link_t path_ping(link_t link, void (*pong)(link_t link, lob_t status, void *arg), void *arg);

lob_t path_on_open(link_t link, lob_t open);

#endif
#ifndef ext_peer_h
#define ext_peer_h

#include "ext.h"

// enables peer/connect handling for this mesh
mesh_t peer_enable(mesh_t mesh);

// become a router
mesh_t peer_route(mesh_t mesh);

// set this link as the default router for any peer
link_t peer_router(link_t router);

// try to connect this peer via this router (sends an ad-hoc peer request)
link_t peer_connect(link_t peer, link_t router);


#endif

#ifndef ext_sock_h
#define ext_sock_h
/*
#include "ext.h"

#define SOCKC_NEW 0
#define SOCKC_OPEN 1
#define SOCKC_CLOSED 2

typedef struct sockc_struct 
{
  uint8_t state;
  uint32_t readable, writing;
  uint8_t *readbuf, *writebuf, *zwrite;
  lob_t opts;
  chan_t c;
  int fd; // convenience for app usage, initialized to -1
} *sockc_t;

// process channel for new incoming sock channel, any returned sockc must have a read or close function called
// state is SOCKC_OPEN or SOCKC_NEW (until sockc_accept())
// it's set to SOCKC_CLOSED after all the data is read on an ended channel or sockc_close() is called
sockc_t ext_sock(chan_t c);

// changes state from SOCKC_NEW to SOCKC_OPEN
void sockc_accept(sockc_t sc);

// create a sock channel to this hn, optional opts (ip, port), sets state=SOCKC_OPEN
sockc_t sockc_connect(switch_t s, char *hn, lob_t opts);

// tries to flush and end, cleans up, sets SOCKC_CLOSED
sockc_t sockc_close(sockc_t sock);

// must be called to free up sc resources (internally calls sockc_close to be sure)
sockc_t sockc_free(sockc_t sc);

// -1 on err, returns bytes read into buf up to len, sets SOCKC_CLOSED when channel ended
int sockc_read(sockc_t sock, uint8_t *buf, int len);

// -1 on err, returns len and will always buffer up to len 
int sockc_write(sockc_t sock, uint8_t *buf, int len);

// general flush outgoing buffer into a packet
void sockc_flush(chan_t c);

// advances readbuf by this len, for use when doing zero-copy reading directly from ->readbuf
// sets SOCKC_CLOSED when channel ended
void sockc_zread(sockc_t sc, int len);

// creates zero-copy buffer space of requested len at sc->zwrite or returns 0
int sockc_zwrite(sockc_t sc, int len);

// must use after writing anything to ->zwrite, adds len to outgoing buffer and resets sc->zwrite, returns len
int sockc_zwritten(sockc_t sc, int len);

// serial-style single character interface
int sockc_available(sockc_t sc);
uint8_t sockc_sread(sockc_t sc);
uint8_t sockc_swrite(sockc_t sc, uint8_t byte);
*/
#endif
#ifndef ext_stream_h
#define ext_stream_h

#include "mesh.h"

// add stream channel support
mesh_t ext_stream(mesh_t mesh);


#endif
#ifndef ext_thtp_h
#define ext_thtp_h

/*
#include "xht.h"
#include "chan.h"

void ext_thtp(chan_t c);

// optionally initialize thtp w/ an index, happens automatically too
void thtp_init(switch_t s, xht_t index);
void thtp_free(switch_t s);

// sends requests matching this glob ("/path" matches "/path/foo") to this note, most specific wins
void thtp_glob(switch_t s, char *glob, lob_t note);

// sends requests matching this exact path to this note
void thtp_path(switch_t s, char *path, lob_t note);

// generate an outgoing request, send the response attached to the note
chan_t thtp_req(switch_t s, lob_t note);
*/
#endif
#ifndef hashname_h
#define hashname_h

#include "base32.h"
#include "lob.h"

// overall type
typedef struct hashname_struct *hashname_t;

// only things that actually malloc/free
hashname_t hashname_dup(hashname_t hn);
hashname_t hashname_free(hashname_t hn);

// everything else returns a pointer to a static global for temporary use
hashname_t hashname_vchar(const char *str); // from a string
hashname_t hashname_vbin(const uint8_t *bin);
hashname_t hashname_vkeys(lob_t keys);
hashname_t hashname_vkey(lob_t key, uint8_t id); // key is body, intermediates in json

// accessors
uint8_t *hashname_bin(hashname_t hn); // 32 bytes
char *hashname_char(hashname_t hn); // 52 byte base32 string w/ \0 (TEMPORARY)
char *hashname_short(hashname_t hn); // 16 byte base32 string w/ \0 (TEMPORARY)

// utilities related to hashnames
int hashname_cmp(hashname_t a, hashname_t b);  // memcmp shortcut
uint8_t hashname_id(lob_t a, lob_t b); // best matching id (single byte)
lob_t hashname_im(lob_t keys, uint8_t id); // intermediate hashes in the json, optional id to set that as body


#endif

// key = string to match or null
// klen = key length (or 0), or if null key then len is the array offset value
// json = json object or array
// jlen = length of json
// vlen = where to store return value length
// returns pointer to value and sets len to value length, or 0 if not found or any error
char *js0n(char *key, size_t klen, char *json, size_t jlen, size_t *vlen);
#ifndef jwt_h
#define jwt_h

#include <stdint.h>
#include "lob.h"
#include "e3x.h"

// utils to parse and generate JWTs to/from LOBs

// one JWT is two chained LOB packets
//  token->head is the JWT header JSON
//  token->body is a raw LOB for the claims
//  claims->head is the JWT claims JSON
//  claims->body is the JWT signature

lob_t jwt_decode(char *encoded, size_t len); // base64
lob_t jwt_parse(uint8_t *raw, size_t len); // from raw lobs
lob_t jwt_claims(lob_t token); // just returns the token->chain

char *jwt_encode(lob_t token); // char* is cached/freed inside token
uint8_t *jwt_raw(lob_t token); // lob-encoded raw bytes of whole thing
uint32_t jwt_len(lob_t token); // length of raw bytes

lob_t jwt_verify(lob_t token, e3x_exchange_t x);
lob_t jwt_sign(lob_t token, e3x_self_t self);

// return >0 if this alg is supported
uint8_t jwt_alg(char *alg);

#endif
#include "base32.h"
#include "base64.h"
#include "hashname.h"
#include "js0n.h"
#include "lob.h"
#include "murmur.h"
#include "chacha.h"
#include "xht.h"
#ifndef link_h
#define link_h
#include <stdint.h>

#include "mesh.h"

struct link_struct
{
  char handle[17]; // b32 hashname_short
  uint8_t csid;

  // public link data
  hashname_t id;
  e3x_exchange_t x;
  mesh_t mesh;
  lob_t key;
  lob_t handshakes;
  chan_t chans;
  
  // these are for internal link management only
  struct seen_struct *pipes;
  link_t next;
};

// these all create or return existing one from the mesh
link_t link_get(mesh_t mesh, hashname_t id);
link_t link_keys(mesh_t mesh, lob_t keys); // adds in the right key
link_t link_key(mesh_t mesh, lob_t key, uint8_t csid); // adds in from the body

// get link info json
lob_t link_json(link_t link);

// removes from mesh
void link_free(link_t link);

// load in the key to existing link
link_t link_load(link_t link, uint8_t csid, lob_t key);

// try to turn a path into a pipe and add it to the link
pipe_t link_path(link_t link, lob_t path);

// just manage a pipe directly, removes if pipe->down, else adds
link_t link_pipe(link_t link, pipe_t pipe);

// iterate through existing pipes for a link
pipe_t link_pipes(link_t link, pipe_t after);

// add a custom outgoing handshake packet for this link
link_t link_handshake(link_t link, lob_t handshake);

// process a decrypted channel packet
link_t link_receive(link_t link, lob_t inner, pipe_t pipe);

// process an incoming handshake
link_t link_receive_handshake(link_t link, lob_t handshake, pipe_t pipe);

// try to deliver this packet to the best pipe
link_t link_send(link_t link, lob_t inner);

// return current handshake(s)
lob_t link_handshakes(link_t link);

// send current handshake(s) to all pipes and return them
lob_t link_sync(link_t link);

// generate new encrypted handshake(s) and sync
lob_t link_resync(link_t link);

// is the other endpoint connected and the link available, NULL if not
link_t link_up(link_t link);

// force link down, ends channels and generates events
link_t link_down(link_t link);

// create/track a new channel for this open
chan_t link_chan(link_t link, lob_t open);

// encrypt and send this one packet on this pipe
link_t link_direct(link_t link, lob_t inner, pipe_t pipe);

// process any channel timeouts based on the current/given time
link_t link_process(link_t link, uint32_t now);

#endif
#ifndef lob_h
#define lob_h

#include <stdint.h>
#include <stdlib.h>

typedef struct lob_struct
{
  // these are public but managed by accessors
  uint8_t *raw;
  uint8_t *body;
  size_t body_len;
  uint8_t *head;
  size_t head_len;
  
  // these are all for external use only
  uint32_t id;
  void *arg;

  // these are internal/private
  struct lob_struct *chain;
  char *cache; // edited copy of the json head

  // used only by the list utils
  struct lob_struct *next, *prev;

} *lob_t;

// these all allocate/free memory
lob_t lob_new();
lob_t lob_copy(lob_t p);
lob_t lob_free(lob_t p); // returns NULL for convenience

// creates a new parent packet chained to the given child one, so freeing the new packet also free's it
lob_t lob_chain(lob_t child);
// manually chain together two packets, returns parent, frees any existing child, creates parent if none
lob_t lob_link(lob_t parent, lob_t child);
// return a linked child if any
lob_t lob_linked(lob_t parent);
// returns child, unlinked
lob_t lob_unlink(lob_t parent);

// initialize head/body from raw, parses json
lob_t lob_parse(const uint8_t *raw, size_t len);

// return full encoded packet
uint8_t *lob_raw(lob_t p);
size_t lob_len(lob_t p);

// return null-terminated json header only
char *lob_json(lob_t p);

// set/store these in the current packet
uint8_t *lob_head(lob_t p, uint8_t *head, size_t len);
uint8_t *lob_body(lob_t p, uint8_t *body, size_t len);
lob_t lob_append(lob_t p, uint8_t *chunk, size_t len);
lob_t lob_append_str(lob_t p, char *chunk);

// convenient json setters/getters, always return given lob so they're chainable
lob_t lob_set_raw(lob_t p, char *key, size_t klen, char *val, size_t vlen); // raw
lob_t lob_set(lob_t p, char *key, char *val); // escapes value
lob_t lob_set_len(lob_t p, char *key, size_t klen, char *val, size_t vlen); // same as lob_set
lob_t lob_set_int(lob_t p, char *key, int val);
lob_t lob_set_uint(lob_t p, char *key, unsigned int val);
lob_t lob_set_float(lob_t p, char *key, float val, uint8_t places);
lob_t lob_set_printf(lob_t p, char *key, const char *format, ...);
lob_t lob_set_base32(lob_t p, char *key, uint8_t *val, size_t vlen);

// copies keys from json into p
lob_t lob_set_json(lob_t p, lob_t json);

// count of keys
unsigned int lob_keys(lob_t p);

// alpha-sorts the json keys
lob_t lob_sort(lob_t p);

// 0 to match, !0 if different, compares only top-level json and body
int lob_cmp(lob_t a, lob_t b);

// the return uint8_t* is invalidated with any _set* operation!
char *lob_get(lob_t p, char *key);
int lob_get_int(lob_t p, char *key);
unsigned int lob_get_uint(lob_t p, char *key);
float lob_get_float(lob_t p, char *key);

char *lob_get_index(lob_t p, uint32_t i); // returns ["0","1","2","3"] or {"0":"1","2":"3"}

// just shorthand for util_cmp to match a key/value
int lob_get_cmp(lob_t p, char *key, char *val);

// get the raw value, must use get_len
char *lob_get_raw(lob_t p, char *key);
size_t lob_get_len(lob_t p, char *key);

// returns new packets based on values
lob_t lob_get_json(lob_t p, char *key); // creates new packet from key:object value
lob_t lob_get_array(lob_t p, char *key); // list of packet->next from key:[object,object]
lob_t lob_get_base32(lob_t p, char *key); // decoded binary is the return body

// handles cloaking conveniently, len is lob_len()+(8*rounds)
uint8_t *lob_cloak(lob_t p, uint8_t rounds);

// decloaks and parses
lob_t lob_decloak(uint8_t *cloaked, size_t len);

// TODO, this would be handy, js syntax to get a json value
// char *lob_eval(lob_t p, "foo.bar[0]['zzz']");

// manage a basic double-linked list of packets using ->next and ->prev
lob_t lob_pop(lob_t list); // returns last item, item->next is the new list
lob_t lob_push(lob_t list, lob_t append); // appends new item, returns new list
lob_t lob_shift(lob_t list); // returns first item, item->next is the new list
lob_t lob_unshift(lob_t list, lob_t prepend); // adds item, returns new list
lob_t lob_splice(lob_t list, lob_t extract); // removes item from list, returns new list
lob_t lob_insert(lob_t list, lob_t after, lob_t p); // inserts item in list after other item, returns new list
lob_t lob_freeall(lob_t list); // frees all
lob_t lob_match(lob_t list, char *key, char *value); // find the first packet in the list w/ the matching key/value
lob_t lob_next(lob_t list);
lob_t lob_array(lob_t list); // return json array of the list

#endif
#ifndef mesh_h
#define mesh_h

typedef struct mesh_struct *mesh_t;
typedef struct link_struct *link_t;
typedef struct pipe_struct *pipe_t;
typedef struct chan_struct *chan_t;



#include "e3x.h"
#include "lib.h"
#include "util.h"
#include "chan.h"
#include "pipe.h"
#include "link.h"

struct mesh_struct
{
  hashname_t id;
  char *uri;
  lob_t keys, paths;
  e3x_self_t self;
  xht_t index; // for extensions to use
  void *on; // internal list of triggers
  // shared network info
  uint16_t port_local, port_public;
  char *ipv4_local, *ipv4_public;
  lob_t handshakes, cached; // handshakes
  void *routes; // internal routing
  link_t links;
};

// pass in a prime for the main index of hashnames+links+channels, 0 to use compiled default
mesh_t mesh_new(uint32_t prime);
mesh_t mesh_free(mesh_t mesh);

// must be called to initialize to a hashname from keys/secrets, return !0 if failed
uint8_t mesh_load(mesh_t mesh, lob_t secrets, lob_t keys);

// creates and loads a new random hashname, returns secrets if it needs to be saved/reused
lob_t mesh_generate(mesh_t mesh);

// return the best current URI to this endpoint, optional base
char *mesh_uri(mesh_t mesh, char *base);

// generate json of mesh keys and current paths
lob_t mesh_json(mesh_t mesh);

// generate json for all links, returns lob list
lob_t mesh_links(mesh_t mesh);

// creates a link from the json format of {"hashname":"...","keys":{},"paths":[]}, optional direct pipe too
link_t mesh_add(mesh_t mesh, lob_t json, pipe_t pipe);

// return only if this hashname (full or short) is currently linked (in any state)
link_t mesh_linked(mesh_t mesh, char *hn, size_t len);
link_t mesh_linkid(mesh_t mesh, hashname_t id); // TODO, clean this up

// remove this link, will event it down and clean up during next process()
mesh_t mesh_unlink(link_t link);

// add a custom outgoing handshake packet to all links
mesh_t mesh_handshake(mesh_t mesh, lob_t handshake);

// query the cache of handshakes for a matching one with a specific type
lob_t mesh_handshakes(mesh_t mesh, lob_t handshake, char *type);

// processes incoming packet, it will take ownership of packet, returns link delivered to if success
link_t mesh_receive(mesh_t mesh, lob_t packet, pipe_t pipe);

// process any unencrypted handshake packet, cache if needed
link_t mesh_receive_handshake(mesh_t mesh, lob_t handshake, pipe_t pipe);

// process any channel timeouts based on the current/given time
mesh_t mesh_process(mesh_t mesh, uint32_t now);

// adds a forwarding route for any incoming packet w/ this token
mesh_t mesh_forward(mesh_t m, uint8_t *token, link_t to, uint8_t flag);

// callback when the mesh is free'd
void mesh_on_free(mesh_t mesh, char *id, void (*free)(mesh_t mesh));

// callback when a path needs to be turned into a pipe
void mesh_on_path(mesh_t mesh, char *id, pipe_t (*path)(link_t link, lob_t path));
pipe_t mesh_path(mesh_t mesh, link_t link, lob_t path);

// callback when an unknown hashname is discovered
void mesh_on_discover(mesh_t mesh, char *id, link_t (*discover)(mesh_t mesh, lob_t discovered, pipe_t pipe));
void mesh_discover(mesh_t mesh, lob_t discovered, pipe_t pipe);

// callback when a link changes state created/up/down
void mesh_on_link(mesh_t mesh, char *id, void (*link)(link_t link));
void mesh_link(mesh_t mesh, link_t link);

// callback when a new incoming channel is requested
void mesh_on_open(mesh_t mesh, char *id, lob_t (*open)(link_t link, lob_t open));
lob_t mesh_open(mesh_t mesh, link_t link, lob_t open);


#endif
// local wrappers/additions
#include <stdint.h>

// murmurhash3 32bit
uint32_t murmur4(const uint8_t *data, uint32_t len);

// hex must be 8+\0
char *murmur8(const uint8_t *data, uint32_t len, char *hex);

// more convenient, caller must ensure 4-byte sizing
uint8_t *murmur(const uint8_t *data, uint32_t len, uint8_t *hash);


/*-----------------------------------------------------------------------------
 * MurmurHash3 was written by Austin Appleby, and is placed in the public
 * domain.
 *
 * This implementation was written by Shane Day, and is also public domain.
 *
 * This is a portable ANSI C implementation of MurmurHash3_x86_32 (Murmur3A)
 * with support for progressive processing.
 */

/* ------------------------------------------------------------------------- */
/* Determine what native type to use for uint32_t */

/* We can't use the name 'uint32_t' here because it will conflict with
 * any version provided by the system headers or application. */

/* First look for special cases */
#if defined(_MSC_VER)
  #define MH_UINT32 unsigned long
#endif

/* If the compiler says it's C99 then take its word for it */
#if !defined(MH_UINT32) && ( \
     defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L )
  #include <stdint.h>
  #define MH_UINT32 uint32_t
#endif

/* Otherwise try testing against max value macros from limit.h */
#if !defined(MH_UINT32)
  #include  <limits.h>
  #if   (USHRT_MAX == 0xffffffffUL)
    #define MH_UINT32 unsigned short
  #elif (UINT_MAX == 0xffffffffUL)
    #define MH_UINT32 unsigned int
  #elif (ULONG_MAX == 0xffffffffUL)
    #define MH_UINT32 unsigned long
  #endif
#endif

#if !defined(MH_UINT32)
  #error Unable to determine type name for unsigned 32-bit int
#endif

/* I'm yet to work on a platform where 'unsigned char' is not 8 bits */
#define MH_UINT8  unsigned char


/* ------------------------------------------------------------------------- */
/* Prototypes */

#ifdef __cplusplus
extern "C" {
#endif

void PMurHash32_Process(MH_UINT32 *ph1, MH_UINT32 *pcarry, const void *key, int len);
MH_UINT32 PMurHash32_Result(MH_UINT32 h1, MH_UINT32 carry, MH_UINT32 total_length);
MH_UINT32 PMurHash32(MH_UINT32 seed, const void *key, int len);

void PMurHash32_test(const void *key, int len, MH_UINT32 seed, void *out);

#ifdef __cplusplus
}
#endif
#ifndef net_loopback_h
#define net_loopback_h

#include "mesh.h"

typedef struct net_loopback_struct
{
  pipe_t pipe;
  mesh_t a, b;
} *net_loopback_t;

// connect two mesh instances with each other for packet delivery
net_loopback_t net_loopback_new(mesh_t a, mesh_t b);
void net_loopback_free(net_loopback_t pair);

#endif
#ifndef net_serial_h
#define net_serial_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#include "mesh.h"

// overall server
typedef struct net_serial_struct
{
  mesh_t mesh;
  xht_t pipes;
} *net_serial_t;

// create a new serial hub
net_serial_t net_serial_new(mesh_t mesh, lob_t options);
void net_serial_free(net_serial_t net);

// add a named serial pipe and I/O callbacks for it
pipe_t net_serial_add(net_serial_t net, const char *name, int (*read)(void), int (*write)(uint8_t *buf, size_t len), uint8_t buffer);

// manually send a packet down a named pipe (discovery, etc)
net_serial_t net_serial_send(net_serial_t net, const char *name, lob_t packet);

// check all pipes for data
net_serial_t net_serial_loop(net_serial_t net);

#ifdef __cplusplus
}
#endif

#endif
#ifndef net_tcp4_h
#define net_tcp4_h

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#include "mesh.h"

// overall server
typedef struct net_tcp4_struct
{
  int server;
  int port;
  mesh_t mesh;
  xht_t pipes;
  lob_t path;
} *net_tcp4_t;

// create a new listening tcp server
net_tcp4_t net_tcp4_new(mesh_t mesh, lob_t options);
void net_tcp4_free(net_tcp4_t net);

// check all sockets for work (just for testing, use libuv or such for production instead)
net_tcp4_t net_tcp4_loop(net_tcp4_t net);

#endif

#endif
#ifndef net_udp4_h
#define net_udp4_h

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#include "mesh.h"

// overall server
typedef struct net_udp4_struct
{
  int server;
  int port;
  mesh_t mesh;
  xht_t pipes;
  lob_t path; // to us
} *net_udp4_t;

// create a new listening udp server
net_udp4_t net_udp4_new(mesh_t mesh, lob_t options);
void net_udp4_free(net_udp4_t net);

// receive a packet into this mesh
net_udp4_t net_udp4_receive(net_udp4_t net);

#endif

#endif
#ifndef pipe_h
#define pipe_h
#include <stdint.h>

#include "mesh.h"
#include "link.h"

struct pipe_struct
{
  lob_t path;
  lob_t links; // who to signal for pipe events
  void *arg; // for use by app/network transport
  pipe_t next; // for transport use
  void (*send)(pipe_t pipe, lob_t packet, link_t link); // deliver this packet via this pipe

  char *type;
  char *id;
  uint8_t down:1;
  uint8_t cloaked:1;
  uint8_t local:1;
};

pipe_t pipe_new(char *type);
pipe_t pipe_free(pipe_t p);

// generates notifications to any links using it
pipe_t pipe_sync(pipe_t p, uint8_t down);

// safe wrapper around ->send
pipe_t pipe_send(pipe_t pipe, lob_t packet, link_t link);

#endif
#ifndef socketio_h
#define socketio_h

// utilities to parse and generate minimal socket.io / engine.io packets

#include <stdint.h>
#include "lob.h"

// https://github.com/Automattic/engine.io-protocol
#define SOCKETIO_ETYPE_OPEN 0
#define SOCKETIO_ETYPE_CLOSE 1
#define SOCKETIO_ETYPE_PING 2
#define SOCKETIO_ETYPE_PONG 3
#define SOCKETIO_ETYPE_MESSAGE 4
#define SOCKETIO_ETYPE_UPGRADE 5
#define SOCKETIO_ETYPE_NOOP 6

// https://github.com/Automattic/socket.io-protocol
#define SOCKETIO_PTYPE_CONNECT 0
#define SOCKETIO_PTYPE_DISCONNECT 1
#define SOCKETIO_PTYPE_EVENT 2
#define SOCKETIO_PTYPE_ACK 3
#define SOCKETIO_PTYPE_ERROR 4
#define SOCKETIO_PTYPE_BINARY_EVENT 5
#define SOCKETIO_PTYPE_BINARY_ACK 6


lob_t socketio_decode(lob_t data);

lob_t socketio_encode(uint8_t etype, uint8_t ptype, lob_t payload);

#endif
#ifndef tmesh_h
#define tmesh_h

#include "mesh.h"

/*

one mote per link
every mote is part of one community
public communities start w/ "Public"
private include hashname in secret, new motes must be invited by active ones
every community has one or more mediums
each community medium has 1+ beacon motes
beacon tx/rx is always fixed channel, public community has dedicated beacon hashname \0\0\0\0...
one beacon mote per known hashname for signal detection and handshakes, cloned for new links
all known links in a community have motes bound to just one medium

mote_sync(a, b) - clones state from a to b
  - use this on any incoming public beacon rx to private beacon tx
  - use to transition from beacon to link mote
*/

typedef struct tmesh_struct *tmesh_t;
typedef struct cmnty_struct *cmnty_t; // list of mediums and beacon mote
typedef struct mote_struct *mote_t; // secret, nonce, time, knock, link
typedef struct medium_struct *medium_t; // channels, energy
typedef struct radio_struct *radio_t; // driver utils
typedef struct knock_struct *knock_t; // single action

// medium management w/ device driver
struct medium_struct
{
  cmnty_t com; // mediums belong to one community
  medium_t next; // for lists
  void *arg; // for use by radio device driver
  uint32_t min, max; // cycles to knock, set by driver
  uint32_t avg; // average actual cycles to tx for drift calc
  uint8_t bin[5];
  uint8_t chans; // number of total channels, set by driver
  uint8_t z; // default
  uint8_t radio:4; // radio device id based on radio_devices[]
  uint8_t zshift:4; // window time exponent offset for this medium
};

// community management
struct cmnty_struct
{
  tmesh_t tm;
  char *name;
  medium_t medium; // TODO, support multiple per community
  mote_t beacons;
  mote_t links;
  pipe_t pipe; // one pipe per community as it's shared performance
  struct cmnty_struct *next;
};

// join a new private/public community
cmnty_t tmesh_join(tmesh_t tm, char *medium, char *name);

// leave any community
tmesh_t tmesh_leave(tmesh_t tm, cmnty_t c);

// add a link already known to be in this community
mote_t tmesh_link(tmesh_t tm, cmnty_t c, link_t link);

// start looking for this hashname in this community, will link once found
mote_t tmesh_seek(tmesh_t tm, cmnty_t c, hashname_t id);

// if there's a mote for this link, return it
mote_t tmesh_mote(tmesh_t tm, link_t link);

// overall tmesh manager
struct tmesh_struct
{
  mesh_t mesh;
  cmnty_t coms;
  lob_t pubim;
  uint32_t last; // last seen cycles for rebasing
  uint32_t epoch; // for relative time into mesh_process
  uint32_t cycles; // remainder for epoch
  knock_t (*sort)(knock_t a, knock_t b);
  uint8_t seed[4]; // random seed for restart detection
  uint8_t z; // our current z-index
};

// create a new tmesh radio network bound to this mesh
tmesh_t tmesh_new(mesh_t mesh, lob_t options);
void tmesh_free(tmesh_t tm);

// process any knock that has been completed by a driver
tmesh_t tmesh_knocked(tmesh_t tm, knock_t k);

// process everything based on current cycle count, optional rebase cycles
tmesh_t tmesh_process(tmesh_t tm, uint32_t at, uint32_t rebase);

// a single knock request ready to go
struct knock_struct
{
  mote_t mote;
  uint32_t start, stop; // requested start/stop times
  uint32_t started, stopped; // actual start/stop times
  uint8_t frame[64];
  uint8_t nonce[8]; // nonce for this knock
  uint8_t chan; // current channel (< med->chans)
  uint8_t rssi; // set by driver only after rx
  // boolean flags for state tracking, etc
  uint8_t tx:1; // tells radio to tx or rx
  uint8_t ready:1; // is ready to transceive
  uint8_t err:1; // failed
};

// mote state tracking
struct mote_struct
{
  medium_t medium;
  link_t link; // only on link motes
  hashname_t beacon; // only on beacon motes
  mote_t next; // for lists
  util_frames_t frames; // r/w frame buffers
  uint32_t txhash, rxhash, cash; // dup detection
  uint32_t at; // cycles until next knock
  uint16_t txz, rxz; // empty tx/rx counts
  uint16_t txs, rxs; // current tx/rx counts
  uint16_t bad; // dropped bad frames
  uint8_t secret[32];
  uint8_t nonce[8];
  uint8_t seed[4]; // last seen seed to detect resets
  uint8_t chan[2];
  uint8_t last, best, worst; // rssi
  uint8_t z;
  uint8_t order:1; // is hashname compare
  uint8_t public:1; // special public beacon mote
  uint8_t priority:3; // next knock priority
};

// these are primarily for internal use

mote_t mote_new(medium_t medium, hashname_t id);
mote_t mote_free(mote_t m);

// resets secret/nonce and to ping mode
mote_t mote_reset(mote_t m);

// advance mote ahead next window
mote_t mote_advance(mote_t m);

// least significant nonce bit sets direction
uint8_t mote_tx(mote_t m);

// next knock init
mote_t mote_knock(mote_t m, knock_t k);

// initiates handshake over beacon mote
mote_t mote_handshake(mote_t m);

// attempt to establish link from a beacon mote
mote_t mote_link(mote_t m);

// process new link data on a mote
mote_t mote_process(mote_t m);

// for tmesh sorting
knock_t knock_sooner(knock_t a, knock_t b);

///////////////////
// radio devices must process all mediums
struct radio_struct
{
  // initialize any medium scheduling time/cost and channels
  medium_t (*init)(radio_t self, medium_t m);

  // when a medium isn't used anymore, let the radio free any associated resources
  medium_t (*free)(radio_t self, medium_t m);

  // called whenever a new knock is ready to be scheduled
  medium_t (*ready)(radio_t self, medium_t m, knock_t knock);
  
  // shared knock between tmesh and driver
  knock_t knock;

  // for use by the radio driver
  void *arg;

  // guid
  uint8_t id:4;
};

#ifndef RADIOS_MAX
#define RADIOS_MAX 1
#endif
extern radio_t radio_devices[]; // all of em

// globally add/set a new device
radio_t radio_device(radio_t device);



#endif
#ifndef util_chunks_h
#define util_chunks_h

#include <stdint.h>
#include "lob.h"

// for list of incoming chunks
typedef struct util_chunk_struct
{
  struct util_chunk_struct *prev;
  uint8_t size;
  uint8_t data[];
} *util_chunk_t;

typedef struct util_chunks_struct
{

  util_chunk_t reading; // stacked linked list of incoming chunks

  lob_t writing;
  size_t writeat; // offset into lob_raw()
  uint16_t waitat; // gets to 256, offset into current chunk
  uint8_t waiting; // current writing chunk size;
  uint8_t readat; // always less than a max chunk, offset into reading

  uint8_t cap;
  uint8_t blocked:1, blocking:1, ack:1, err:1; // bool flags
} *util_chunks_t;


// size of each chunk, 0 == MAX (256)
util_chunks_t util_chunks_new(uint8_t size);

util_chunks_t util_chunks_free(util_chunks_t chunks);

// turn this packet into chunks and append, free's out
util_chunks_t util_chunks_send(util_chunks_t chunks, lob_t out);

// get any packets that have been reassembled from incoming chunks
lob_t util_chunks_receive(util_chunks_t chunks);

// bytes waiting to be sent
uint32_t util_chunks_writing(util_chunks_t chunks);


////// these are for a stream-based transport

// how many bytes are there ready to write
uint32_t util_chunks_len(util_chunks_t chunks);

// return the next block of data to be written to the stream transport, max len is util_chunks_len()
uint8_t *util_chunks_write(util_chunks_t chunks);

// advance the write this far, don't mix with util_chunks_out() usage
util_chunks_t util_chunks_written(util_chunks_t chunks, size_t len);

// queues incoming stream based data
util_chunks_t util_chunks_read(util_chunks_t chunks, uint8_t *block, size_t len);

////// these are for frame-based transport

// size of the next chunk, -1 when none, max is chunks size-1
int16_t util_chunks_size(util_chunks_t chunks);

// return the next chunk of data, use util_chunks_next to advance
uint8_t *util_chunks_frame(util_chunks_t chunks);

// peek into what the next chunk size will be, to see terminator ones
int16_t util_chunks_peek(util_chunks_t chunks);

// process incoming chunk
util_chunks_t util_chunks_chunk(util_chunks_t chunks, uint8_t *chunk, int16_t size);

// advance the write past the current chunk
util_chunks_t util_chunks_next(util_chunks_t chunks);


#endif
#ifndef util_frames_h
#define util_frames_h

#include <stdint.h>
#include "lob.h"

// for list of incoming frames
typedef struct util_frame_struct
{
  struct util_frame_struct *prev;
  uint8_t data[];
} *util_frame_t;

typedef struct util_frames_struct
{

  lob_t inbox; // received packets waiting to be processed
  lob_t outbox; // current packet being sent out

  util_frame_t cache; // stacked linked list of incoming frames in progress

  uint32_t inlast; // last good incoming frame hash
  uint32_t outbase; // last sent outbox frame hash

  uint8_t in; // number of incoming frames received/waiting
  uint8_t out; //  number of outgoing frames of outbox sent since outbase

  uint8_t size; // frame size
  uint8_t flush:1; // bool to signal a flush is needed
  uint8_t err:1; // unrecoverable failure

} *util_frames_t;


// size of each frame, min 16 max 128, multiple of 4
util_frames_t util_frames_new(uint8_t size);

util_frames_t util_frames_free(util_frames_t frames);

// turn this packet into frames and append, free's out
util_frames_t util_frames_send(util_frames_t frames, lob_t out);

// get any packets that have been reassembled from incoming frames
lob_t util_frames_receive(util_frames_t frames);

// total bytes in the inbox/outbox
size_t util_frames_inlen(util_frames_t frames);
size_t util_frames_outlen(util_frames_t frames);

// the next frame of data in/out, if data NULL return is just ready check bool
util_frames_t util_frames_inbox(util_frames_t frames, uint8_t *data);
util_frames_t util_frames_outbox(util_frames_t frames, uint8_t *data);

// is there an expectation of an incoming frame
util_frames_t util_frames_await(util_frames_t frames);

#endif
#ifndef util_h
#define util_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "util_sys.h"
#include "util_uri.h"
#include "util_chunks.h"
#include "util_frames.h"
#include "util_unix.h"

// make sure out is 2*len + 1
char *util_hex(uint8_t *in, size_t len, char *out);
// out must be len/2
uint8_t *util_unhex(char *in, size_t len, uint8_t *out);
// hex string validator, NULL is invalid, else returns str
char *util_ishex(char *str, uint32_t len);

// safer string comparison (0 == same)
int util_cmp(char *a, char *b);

// portable sort
void util_sort(void *base, unsigned int nel, unsigned int width, int (*comp)(void *, const void *, const void *), void *arg);

// portable reallocf
void *util_reallocf(void *ptr, size_t size);

// get a "now" timestamp to do millisecond timers
uint64_t util_at(void); // only pass at into _since()
uint32_t util_since(uint64_t at); // get ms since the at

// Use a constant time comparison function to avoid timing attacks
int util_ct_memcmp(const void* s1, const void* s2, size_t n);

// embedded may not have strdup but it's a kinda handy shortcut
char *util_strdup(const char *str);
#ifndef strdup
#define strdup util_strdup
#endif

#endif
#ifndef util_sys_h
#define util_sys_h

typedef uint32_t at_t;

// returns a number that increments in seconds for comparison (epoch or just since boot)
at_t util_sys_seconds();

// number of milliseconds since given epoch seconds value
unsigned long long util_sys_ms(long epoch);

unsigned short util_sys_short(unsigned short x);
unsigned long util_sys_long(unsigned long x);

// use the platform's best RNG
void util_sys_random_init(void);
long util_sys_random(void);

// -1 toggles debug, 0 disable, 1 enable
void util_sys_logging(int enabled);

// returns NULL for convenient return logging
void *util_sys_log(const char *file, int line, const char *function, const char * format, ...);

#ifdef NOLOG
#define LOG(...) NULL
#else
#define LOG(fmt, ...) util_sys_log(__FILE__, __LINE__, __func__, fmt, ## __VA_ARGS__)
#endif


#endif
#ifndef util_unix_h
#define util_unix_h

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include "mesh.h"

// load a json file into packet
lob_t util_fjson(char *file);

// load an array of hashnames from a file and add them as links
mesh_t util_links(mesh_t mesh, char *file);

// simple sockets simpler
int util_sock_timeout(int sock, uint32_t ms); // blocking timeout

#endif

#endif
#ifndef util_uri_h
#define util_uri_h

#include <stdint.h>
#include "lob.h"

// utils to get link info in/out of a uri

// parser
lob_t util_uri_parse(char *string);

// get keys from query
lob_t util_uri_keys(lob_t uri);

// get paths from host and query
lob_t util_uri_paths(lob_t uri); // chain

// validate any fragment from this peer
uint8_t util_uri_check(lob_t uri, uint8_t *peer);

// generators
lob_t util_uri_add_keys(lob_t uri, lob_t keys);
lob_t util_uri_add_path(lob_t uri, lob_t path);
lob_t util_uri_add_check(lob_t uri, uint8_t *peer, uint8_t *data, size_t len);
lob_t util_uri_add_data(lob_t uri, uint8_t *data, size_t len);

// serialize out from lob format to "uri" key and return it
char *util_uri_format(lob_t uri);

#endif
#ifndef version_h
#define version_h

#define TELEHASH_VERSION_MAJOR 3
#define TELEHASH_VERSION_MINOR 1
#define TELEHASH_VERSION_PATCH 2
#define TELEHASH_VERSION ((TELEHASH_VERSION_MAJOR) * 10000 + (TELEHASH_VERSION_MINOR) * 100 + (TELEHASH_VERSION_PATCH))

#endif
#include <stddef.h>

#ifndef xht_h
#define xht_h

// simple string->void* hashtable, very static and bare minimal, but efficient

typedef struct xht_struct *xht_t;

// must pass a prime#
xht_t xht_new(unsigned int prime);

// caller responsible for key storage, no copies made (don't free it b4 xht_free()!)
// set val to NULL to clear an entry, memory is reused but never free'd (# of keys only grows to peak usage)
void xht_set(xht_t h, const char *key, void *val);

// ooh! unlike set where key/val is in caller's mem, here they are copied into xht_t and free'd when val is 0 or xht_free()
void xht_store(xht_t h, const char *key, void *val, size_t vlen);

// returns value of val if found, or NULL
void *xht_get(xht_t h, const char *key);

// free the hashtable and all entries
void xht_free(xht_t h);

// pass a function that is called for every key that has a value set
typedef void (*xht_walker)(xht_t h, const char *key, void *val, void *arg);
void xht_walk(xht_t h, xht_walker w, void *arg);

// iterator through all the keys (NULL to start), use get for values
char *xht_iter(xht_t h, char *key);

#endif

