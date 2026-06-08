const process = require("process");
const Bun = require("bun");

const BABEL_MAX_NAME_LENGTH = 2n;
const BABEL_ALPHABET_LENGTH = 70n;
const BABEL_DIRECTORY_OBJECTS = BABEL_ALPHABET_LENGTH ** BABEL_MAX_NAME_LENGTH;

const alphabet = [
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
  'w', 'x', 'y', 'z', '0', '1', '2', '3',
  '4', '5', '6', '7', '8', '9', '!', ' ',
  '&', '(', ')', '-', '_', '+', '[', ']',
  '^', '='
];

const filePath = process.argv[2];
if (!filePath) {
  console.error("No file specified!");
  process.exit(1);
}

const file = Bun.file(filePath);
if (!(await file.exists())) {
  console.error("File not found!");
  process.exit(1);
}

const bytes = await file.bytes();

let index = 0n;
for (let i = 0n; i < BigInt(file.size); i ++) {
  index += (BigInt(bytes[i]) + 1n) * (256n ** i);
}

let path = "";
let lastSlash = 0;

while (index) {
  index --;
  let handle = index % BABEL_DIRECTORY_OBJECTS;
  for (let i = 0; i < BABEL_MAX_NAME_LENGTH; i ++) {
    path = alphabet[handle % BABEL_ALPHABET_LENGTH] + path;
    handle /= BABEL_ALPHABET_LENGTH;
  }
  path = "/" + path;
  index /= BABEL_DIRECTORY_OBJECTS;
}

console.log(path);
