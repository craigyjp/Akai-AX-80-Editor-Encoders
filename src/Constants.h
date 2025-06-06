const char* VERSION = "V1.1";

#define RE_READ -9
#define NO_OF_PARAMS 50
const char* INITPATCHNAME = "Initial Patch";
#define HOLD_DURATION 1000
const uint32_t CLICK_DURATION = 250;
#define PATCHES_LIMIT 999
String INITPATCH = "A Piano, 8, 1, 99, 4, 1, 99, 34, 51, 1, 0, 49, 2, 32, 63, 8, 57, 16, 58, 11, 1, 0, 0, 4, 0, 17, 0, 3, 0, 0, 0, 4, 1, 10, 54, 0, 11, 50, 14, 23, 11, 99, 47, 1, 99, 85";

// Create and populate the array with data

const String factoryKL[32] = {
  "A Piano, 8, 1, 99, 4, 1, 99, 34, 51, 1, 0, 49, 2, 32, 63, 8, 57, 16, 58, 11, 1, 0, 0, 4, 0, 17, 0, 3, 0, 0, 0, 4, 1, 10, 54, 0, 11, 50, 14, 23, 11, 99, 47, 1, 99, 85",
  "E Piano, 8, 2, 34, 10, 0, 58, 36, 50, 1, 0, 50, 1, 99, 53, 10, 53, 58, 87, 15, 7, 12, 0, 4, 0, 12, 0, 4, 0, 20, 0, 4, 1, 3, 43, 0, 12, 63, 0, 4, 0, 16, 0, 1, 99, 99",
  "Clavichord, 16, 2, 99, 0, 0, 99, 4, 71, 1, 1, 78, 1, 32, 50, 23, 75, 23, 35, 0, 0, 52, 2, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 0, 44, 0, 0, 0, 0, 0, 70, 0, 0, 1, 91, 99",
  "Harpsichord, 8, 3, 99, 0, 1, 82, 19, 29, 3, 1, 42, 1, 99, 74, 32, 80, 99, 1, 0, 0, 49, 0, 4, 0, 43, 0, 2, 0, 61, 0, 2, 1, 0, 27, 0, 5, 53, 0, 97, 0, 0, 87, 1, 54, 26",
  "Toy Piano, 4, 0, 0, 0, 1, 99, 36, 47, 2, 0, 72, 0, 99, 70, 6, 0, 57, 0, 99, 16, 0, 0, 4, 0, 81, 0, 3, 0, 44, 0, 1, 1, 1, 39, 0, 25, 71, 0, 0, 0, 0, 48, 1, 57, 77",
  "Rag Piano, 8, 3, 24, 1, 1, 99, 19, 57, 1, 2, 50, 2, 57, 55, 5, 70, 3, 38, 5, 7, 52, 0, 4, 0, 90, 0, 3, 0, 24, 0, 4, 1, 0, 52, 0, 19, 19, 0, 24, 0, 9, 0, 1, 88, 32",
  "Trumpet 1, 8, 3, 99, 1, 0, 99, 31, 45, 1, 1, 50, 1, 35, 31, 15, 77, 23, 0, 0, 0, 40, 9, 4, 24, 9, 0, 4, 11, 0, 10, 4, 1, 30, 65, 91, 12, 50, 31, 65, 97, 11, 91, 1, 0, 27",
  "Trumpet 2, 8, 1, 0, 50, 0, 99, 12, 60, 1, 0, 50, 2, 54, 59, 12, 77, 13, 34, 11, 0, 89, 0, 4, 24, 0, 0, 4, 0, 58, 0, 4, 1, 37, 69, 72, 12, 99, 49, 47, 53, 9, 91, 1, 72, 62",

  "Flugle Horn, 16, 1, 0, 50, 0, 98, 0, 60, 1, 0, 50, 2, 54, 48, 14, 70, 23, 4, 11, 0, 49, 0, 4, 0, 0, 0, 4, 0, 58, 0, 4, 1, 42, 69, 72, 11, 99, 49, 47, 53, 9, 91, 1, 72, 75",
  "Horn, 16, 1, 0, 0, 0, 99, 0, 50, 0, 0, 50, 1, 0, 19, 10, 85, 40, 37, 0, 0, 31, 9, 4, 3, 26, 64, 4, 0, 20, 15, 4, 1, 35, 83, 68, 36, 34, 37, 66, 55, 16, 71, 1, 55, 80",
  "Flute, 4, 1, 0, 0, 0, 99, 24, 45, 1, 0, 50, 1, 99, 7, 4, 82, 40, 50, 28, 8, 42, 9, 4, 3, 21, 0, 3, 0, 0, 0, 1, 1, 38, 22, 89, 8, 50, 0, 4, 97, 11, 91, 2, 0, 40",
  "Clarinet, 8, 2, 0, 0, 0, 98, 0, 50, 0, 0, 50, 2, 0, 26, 2, 74, 42, 0, 0, 13, 33, 26, 4, 0, 0, 0, 4, 0, 0, 0, 1, 1, 41, 87, 80, 9, 31, 35, 95, 73, 9, 9, 1, 28, 80",
  "Sax, 16, 3, 88, 0, 0, 99, 24, 50, 2, 1, 56, 1, 52, 40, 29, 69, 14, 26, 8, 8, 40, 9, 4, 3, 21, 0, 4, 0, 0, 0, 1, 1, 20, 63, 61, 3, 50, 2, 99, 94, 12, 91, 1, 14, 33",
  "Strings, 8, 2, 55, 4, 0, 98, 12, 42, 1, 0, 50, 2, 56, 67, 10, 50, 59, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 51, 30, 99, 30, 0, 0, 95, 99, 99, 0, 1, 37, 50",
  "String Ens., 16, 2, 55, 4, 0, 90, 0, 67, 1, 2, 50, 2, 56, 69, 3, 50, 15, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 1, 1, 38, 30, 99, 34, 0, 0, 99, 99, 99, 0, 1, 37, 94",
  "Solo Violin, 8, 3, 35, 3, 0, 84, 24, 59, 1, 0, 50, 2, 62, 77, 0, 59, 15, 89, 0, 5, 48, 27, 4, 0, 59, 52, 2, 0, 0, 0, 4, 1, 36, 54, 54, 14, 0, 39, 48, 50, 53, 0, 1, 37, 38",

  "Electric Bass, 16, 3, 87, 3, 1, 98, 0, 52, 0, 2, 50, 2, 99, 30, 17, 72, 99, 99, 0, 0, 52, 0, 4, 0, 90, 0, 3, 2, 4, 0, 4, 1, 16, 56, 0, 0, 0, 9, 31, 0, 0, 0, 1, 48, 39",
  "Bass, 16, 1, 99, 3, 1, 99, 12, 52, 1, 0, 50, 2, 99, 36, 30, 59, 85, 99, 0, 0, 52, 0, 4, 0, 90, 0, 3, 2, 4, 0, 4, 1, 0, 78, 26, 0, 0, 0, 31, 0, 0, 0, 1, 0, 35",
  "Wood Bass, 16, 1, 0, 81, 1, 87, 26, 59, 2, 1, 50, 2, 94, 23, 26, 69, 2, 55, 0, 0, 0, 0, 4, 0, 0, 0, 2, 0, 0, 0, 2, 1, 0, 37, 0, 36, 0, 0, 43, 22, 30, 64, 1, 47, 53",
  "PercHammond, 16, 2, 55, 7, 0, 99, 31, 50, 2, 0, 50, 2, 67, 45, 17, 88, 35, 0, 10, 1, 33, 0, 4, 3, 26, 0, 4, 0, 57, 0, 4, 1, 17, 99, 99, 4, 99, 0, 0, 39, 14, 0, 1, 0, 35",
  "Hammond, 16, 2, 55, 7, 0, 99, 31, 50, 2, 0, 50, 2, 87, 59, 0, 51, 63, 0, 10, 1, 33, 0, 4, 3, 26, 0, 4, 0, 57, 0, 4, 1, 17, 99, 99, 4, 99, 0, 0, 38, 14, 0, 1, 0, 48",
  "Large Organ, 8, 2, 55, 4, 1, 90, 36, 42, 1, 0, 50, 2, 56, 20, 11, 74, 99, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 39, 8, 99, 13, 89, 0, 95, 99, 99, 0, 1, 96, 46",
  "Small Organ, 8, 2, 55, 4, 0, 90, 36, 41, 1, 0, 50, 2, 56, 14, 11, 74, 99, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 39, 8, 99, 14, 99, 0, 96, 99, 88, 0, 1, 96, 77",
  "60s Organ, 8, 2, 55, 7, 1, 88, 7, 50, 2, 1, 50, 2, 93, 44, 17, 72, 70, 1, 10, 0, 33, 0, 4, 0, 26, 0, 4, 0, 41, 0, 4, 1, 0, 74, 69, 0, 7, 0, 0, 13, 14, 0, 1, 0, 39",

  "Orchestron, 8, 1, 18, 6, 1, 99, 12, 40, 1, 0, 50, 1, 99, 58, 0, 96, 0, 99, 0, 0, 0, 0, 4, 0, 0, 0, 4, 5, 0, 0, 4, 1, 1, 71, 71, 28, 0, 0, 23, 11, 16, 0, 2, 99, 44",
  "V. Halen Synth, 16, 1, 0, 0, 0, 99, 0, 60, 1, 0, 50, 2, 54, 65, 0, 54, 43, 0, 0, 0, 99, 0, 4, 0, 0, 0, 4, 0, 58, 0, 4, 1, 0, 69, 72, 14, 99, 22, 7, 6, 9, 91, 1, 26, 52",
  "Solo Synth, 16, 2, 55, 7, 0, 64, 36, 50, 0, 0, 0, 1, 16, 36, 30, 72, 44, 1, 10, 4, 33, 0, 2, 3, 28, 0, 4, 0, 99, 15, 2, 1, 0, 74, 69, 0, 7, 0, 63, 39, 16, 0, 1, 0, 70",
  "Synth Clav, 16, 1, 99, 0, 0, 79, 12, 45, 1, 0, 50, 2, 53, 51, 26, 80, 61, 50, 0, 0, 52, 0, 4, 0, 4, 0, 3, 35, 4, 0, 4, 1, 0, 33, 0, 0, 0, 0, 0, 21, 0, 0, 1, 37, 79",
  "Synth in 5th, 8, 2, 55, 4, 1, 89, 7, 67, 1, 0, 50, 2, 54, 51, 99, 99, 28, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 10, 85, 99, 7, 0, 0, 96, 99, 99, 0, 1, 37, 39",
  "Spaced, 8, 2, 52, 25, 0, 99, 12, 75, 2, 0, 50, 2, 50, 40, 30, 51, 88, 40, 10, 0, 33, 0, 1, 3, 26, 0, 4, 3, 50, 0, 4, 1, 64, 88, 89, 50, 99, 39, 92, 62, 57, 0, 1, 0, 30",
  "Helicopter, 16, 3, 81, 0, 0, 61, 36, 1, 0, 0, 50, 2, 0, 30, 33, 50, 81, 0, 99, 99, 99, 0, 1, 99, 89, 81, 4, 29, 75, 0, 4, 1, 0, 66, 59, 85, 61, 0, 0, 81, 65, 81, 1, 81, 99",
  "Sweep, 16, 3, 0, 0, 1, 5, 0, 50, 3, 2, 49, 2, 99, 48, 37, 88, 0, 0, 54, 0, 99, 0, 4, 0, 0, 0, 4, 2, 7, 0, 4, 1, 0, 99, 99, 99, 0, 99, 99, 99, 99, 2, 1, 0, 58",
};

const String factoryI[32] = {
  "A Piano, 8, 1, 99, 4, 1, 99, 34, 51, 1, 0, 49, 2, 32, 63, 8, 57, 16, 58, 11, 1, 0, 0, 4, 0, 17, 0, 3, 0, 0, 0, 4, 1, 10, 54, 0, 11, 50, 14, 23, 11, 99, 47, 1, 99, 85",
  "E Piano, 8, 2, 34, 10, 0, 58, 36, 50, 1, 0, 50, 1, 99, 53, 10, 53, 58, 87, 15, 7, 12, 0, 4, 0, 12, 0, 4, 0, 20, 0, 4, 1, 3, 43, 0, 12, 63, 0, 4, 0, 16, 0, 1, 99, 99",
  "Clavichord, 16, 2, 99, 0, 0, 99, 4, 71, 1, 1, 78, 1, 32, 50, 23, 75, 23, 35, 0, 0, 52, 2, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 0, 44, 0, 0, 0, 0, 0, 70, 0, 0, 1, 91, 99",
  "Harpsichord, 8, 3, 99, 0, 1, 82, 19, 29, 3, 1, 42, 1, 99, 74, 32, 80, 99, 1, 0, 0, 49, 0, 4, 0, 43, 0, 2, 0, 61, 0, 2, 1, 0, 27, 0, 5, 53, 0, 97, 0, 0, 87, 1, 54, 26",
  "Toy Piano, 4, 0, 0, 0, 1, 99, 36, 47, 2, 0, 72, 0, 99, 70, 6, 0, 57, 0, 99, 16, 0, 0, 4, 0, 81, 0, 3, 0, 44, 0, 1, 1, 1, 39, 0, 25, 71, 0, 0, 0, 0, 48, 1, 57, 77",
  "Rag Piano, 8, 3, 24, 1, 1, 99, 19, 57, 1, 2, 50, 2, 57, 55, 5, 70, 3, 38, 5, 7, 52, 0, 4, 0, 90, 0, 3, 0, 24, 0, 4, 1, 0, 52, 0, 19, 19, 0, 24, 0, 9, 0, 1, 88, 32",
  "Trumpet 1, 8, 3, 99, 1, 0, 99, 31, 45, 1, 1, 50, 1, 35, 31, 15, 77, 23, 0, 0, 0, 40, 9, 4, 24, 9, 0, 4, 11, 0, 10, 4, 1, 30, 65, 91, 12, 50, 31, 65, 97, 11, 91, 1, 0, 27",
  "Trumpet 2, 8, 1, 0, 50, 0, 99, 12, 60, 1, 0, 50, 2, 54, 59, 12, 77, 13, 34, 11, 0, 89, 0, 4, 24, 0, 0, 4, 0, 58, 0, 4, 1, 37, 69, 72, 12, 99, 49, 47, 53, 9, 91, 1, 72, 62",

  "Flugle Horn, 16, 1, 0, 50, 0, 98, 0, 60, 1, 0, 50, 2, 54, 48, 14, 70, 23, 4, 11, 0, 49, 0, 4, 0, 0, 0, 4, 0, 58, 0, 4, 1, 42, 69, 72, 11, 99, 49, 47, 53, 9, 91, 1, 72, 75",
  "Horn, 16, 1, 0, 0, 0, 99, 0, 50, 0, 0, 50, 1, 0, 19, 10, 85, 40, 37, 0, 0, 31, 9, 4, 3, 26, 64, 4, 0, 20, 15, 4, 1, 35, 83, 68, 36, 34, 37, 66, 55, 16, 71, 1, 55, 80",
  "Flute, 4, 1, 0, 0, 0, 99, 24, 45, 1, 0, 50, 1, 99, 7, 4, 82, 40, 50, 28, 8, 42, 9, 4, 3, 21, 0, 3, 0, 0, 0, 1, 1, 38, 22, 89, 8, 50, 0, 4, 97, 11, 91, 2, 0, 40",
  "Clarinet, 8, 2, 0, 0, 0, 98, 0, 50, 0, 0, 50, 2, 0, 26, 2, 74, 42, 0, 0, 13, 33, 26, 4, 0, 0, 0, 4, 0, 0, 0, 1, 1, 41, 87, 80, 9, 31, 35, 95, 73, 9, 9, 1, 28, 80",
  "Sax, 16, 3, 88, 0, 0, 99, 24, 50, 2, 1, 56, 1, 52, 40, 29, 69, 14, 26, 8, 8, 40, 9, 4, 3, 21, 0, 4, 0, 0, 0, 1, 1, 20, 63, 61, 3, 50, 2, 99, 94, 12, 91, 1, 14, 33",
  "Strings, 8, 2, 55, 4, 0, 98, 12, 42, 1, 0, 50, 2, 56, 67, 10, 50, 59, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 51, 30, 99, 30, 0, 0, 95, 99, 99, 0, 1, 37, 50",
  "String Ens., 16, 2, 55, 4, 0, 90, 0, 67, 1, 2, 50, 2, 56, 69, 3, 50, 15, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 1, 1, 38, 30, 99, 34, 0, 0, 99, 99, 99, 0, 1, 37, 94",
  "Solo Violin, 8, 3, 35, 3, 0, 84, 24, 59, 1, 0, 50, 2, 62, 77, 0, 59, 15, 89, 0, 5, 48, 27, 4, 0, 59, 52, 2, 0, 0, 0, 4, 1, 36, 54, 54, 14, 0, 39, 48, 50, 53, 0, 1, 37, 38",

  "Electric Bass, 16, 3, 87, 3, 1, 98, 0, 52, 0, 2, 50, 2, 99, 30, 17, 72, 99, 99, 0, 0, 52, 0, 4, 0, 90, 0, 3, 2, 4, 0, 4, 1, 16, 56, 0, 0, 0, 9, 31, 0, 0, 0, 1, 48, 39",
  "Bass, 16, 1, 99, 3, 1, 99, 12, 52, 1, 0, 50, 2, 99, 36, 30, 59, 85, 99, 0, 0, 52, 0, 4, 0, 90, 0, 3, 2, 4, 0, 4, 1, 0, 78, 26, 0, 0, 0, 31, 0, 0, 0, 1, 0, 35",
  "Wood Bass, 16, 1, 0, 81, 1, 87, 26, 59, 2, 1, 50, 2, 94, 23, 26, 69, 2, 55, 0, 0, 0, 0, 4, 0, 0, 0, 2, 0, 0, 0, 2, 1, 0, 37, 0, 36, 0, 0, 43, 22, 30, 64, 1, 47, 53",
  "PercHammond, 16, 2, 55, 7, 0, 99, 31, 50, 2, 0, 50, 2, 67, 45, 17, 88, 35, 0, 10, 1, 33, 0, 4, 3, 26, 0, 4, 0, 57, 0, 4, 1, 17, 99, 99, 4, 99, 0, 0, 39, 14, 0, 1, 0, 35",
  "Hammond, 16, 2, 55, 7, 0, 99, 31, 50, 2, 0, 50, 2, 87, 59, 0, 51, 63, 0, 10, 1, 33, 0, 4, 3, 26, 0, 4, 0, 57, 0, 4, 1, 17, 99, 99, 4, 99, 0, 0, 38, 14, 0, 1, 0, 48",
  "Large Organ, 8, 2, 55, 4, 1, 90, 36, 42, 1, 0, 50, 2, 56, 20, 11, 74, 99, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 39, 8, 99, 13, 89, 0, 95, 99, 99, 0, 1, 96, 46",
  "Small Organ, 8, 2, 55, 4, 0, 90, 36, 41, 1, 0, 50, 2, 56, 14, 11, 74, 99, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 39, 8, 99, 14, 99, 0, 96, 99, 88, 0, 1, 96, 77",
  "60s Organ, 8, 2, 55, 7, 1, 88, 7, 50, 2, 1, 50, 2, 93, 44, 17, 72, 70, 1, 10, 0, 33, 0, 4, 0, 26, 0, 4, 0, 41, 0, 4, 1, 0, 74, 69, 0, 7, 0, 0, 13, 14, 0, 1, 0, 39",

  "Orchestron, 8, 1, 18, 6, 1, 99, 12, 40, 1, 0, 50, 1, 99, 58, 0, 96, 0, 99, 0, 0, 0, 0, 4, 0, 0, 0, 4, 5, 0, 0, 4, 1, 1, 71, 71, 28, 0, 0, 23, 11, 16, 0, 2, 99, 44",
  "V. Halen Synth, 16, 1, 0, 0, 0, 99, 0, 60, 1, 0, 50, 2, 54, 65, 0, 54, 43, 0, 0, 0, 99, 0, 4, 0, 0, 0, 4, 0, 58, 0, 4, 1, 0, 69, 72, 14, 99, 22, 7, 6, 9, 91, 1, 26, 52",
  "Solo Synth, 16, 2, 55, 7, 0, 64, 36, 50, 0, 0, 0, 1, 16, 36, 30, 72, 44, 1, 10, 4, 33, 0, 2, 3, 28, 0, 4, 0, 99, 15, 2, 1, 0, 74, 69, 0, 7, 0, 63, 39, 16, 0, 1, 0, 70",
  "Synth Clav, 16, 1, 99, 0, 0, 79, 12, 45, 1, 0, 50, 2, 53, 51, 26, 80, 61, 50, 0, 0, 52, 0, 4, 0, 4, 0, 3, 35, 4, 0, 4, 1, 0, 33, 0, 0, 0, 0, 0, 21, 0, 0, 1, 37, 79",
  "Synth in 5th, 8, 2, 55, 4, 1, 89, 7, 67, 1, 0, 50, 2, 54, 51, 99, 99, 28, 1, 0, 0, 52, 0, 4, 0, 90, 0, 3, 0, 0, 0, 4, 1, 10, 85, 99, 7, 0, 0, 96, 99, 99, 0, 1, 37, 39",
  "Spaced, 8, 2, 52, 25, 0, 99, 12, 75, 2, 0, 50, 2, 50, 40, 30, 51, 88, 40, 10, 0, 33, 0, 1, 3, 26, 0, 4, 3, 50, 0, 4, 1, 64, 88, 89, 50, 99, 39, 92, 62, 57, 0, 1, 0, 30",
  "Helicopter, 16, 3, 81, 0, 0, 61, 36, 1, 0, 0, 50, 2, 0, 30, 33, 50, 81, 0, 99, 99, 99, 0, 1, 99, 89, 81, 4, 29, 75, 0, 4, 1, 0, 66, 59, 85, 61, 0, 0, 81, 65, 81, 1, 81, 99",
  "Sweep, 16, 3, 0, 0, 1, 5, 0, 50, 3, 2, 49, 2, 99, 48, 37, 88, 0, 0, 54, 0, 99, 0, 4, 0, 0, 0, 4, 2, 7, 0, 4, 1, 0, 99, 99, 99, 0, 99, 99, 99, 99, 2, 1, 0, 58",
};

const String KLRAM[64] = {
  "A Piano 16, 0, 1, 1, 99, 4, 1, 99, 35, 51, 1, 0, 49, 1, 32, 48, 8, 53, 90, 99, 26, 0, 0, 0, 3, 0, 10, 36, 0, 12, 90, 0, 99, 99, 0, 17, 0, 2, 0, 0, 0, 3, 14, 23, 11, 99, 0",
  "Piano 8, 1, 1, 3, 99, 0, 1, 99, 24, 60, 2, 0, 50, 1, 57, 28, 16, 69, 34, 0, 33, 7, 52, 0, 3, 0, 0, 12, 0, 11, 18, 0, 37, 99, 0, 90, 0, 2, 0, 0, 0, 3, 0, 19, 65, 49, 0",
  "Clavinet, 2, 1, 3, 78, 0, 0, 82, 19, 55, 2, 2, 50, 0, 99, 99, 22, 38, 43, 0, 47, 0, 0, 0, 3, 0, 17, 9, 0, 9, 99, 0, 37, 99, 0, 43, 0, 1, 0, 81, 0, 1, 10, 19, 99, 99, 0",
  "Harpsichord 2, 3, 1, 3, 99, 0, 0, 82, 17, 44, 3, 1, 40, 0, 99, 99, 41, 42, 99, 0, 0, 0, 0, 0, 3, 0, 0, 30, 0, 4, 53, 0, 54, 31, 0, 43, 0, 1, 0, 81, 0, 1, 0, 97, 0, 0, 87",
  "Jazz Guitar, 4, 2, 2, 21, 24, 0, 29, 36, 50, 2, 0, 50, 1, 99, 51, 0, 68, 21, 14, 23, 0, 51, 0, 3, 0, 0, 55, 99, 37, 0, 0, 37, 62, 0, 90, 0, 2, 0, 57, 0, 3, 0, 35, 9, 74, 0",
  "Rhodes, 5, 1, 2, 47, 0, 0, 99, 12, 57, 2, 0, 50, 1, 57, 64, 6, 70, 3, 38, 23, 7, 52, 0, 3, 0, 0, 33, 0, 19, 19, 0, 88, 79, 0, 90, 0, 2, 0, 0, 0, 3, 0, 24, 11, 9, 0",
  "French Horn, 6, 1, 1, 0, 50, 0, 99, 12, 60, 1, 0, 50, 1, 54, 43, 12, 77, 13, 36, 11, 0, 39, 0, 3, 0, 37, 69, 72, 12, 99, 0, 72, 59, 0, 0, 0, 3, 0, 0, 0, 3, 41, 47, 53, 9, 91",
  "Synth Horn, 7, 0, 3, 2, 0, 0, 99, 24, 50, 1, 1, 50, 0, 45, 35, 22, 74, 26, 18, 13, 8, 40, 10, 3, 0, 25, 63, 81, 1, 50, 0, 0, 41, 3, 21, 0, 3, 0, 0, 0, 0, 2, 51, 99, 8, 91",
  "Soundtrack, 8, 1, 2, 75, 12, 0, 99, 31, 50, 1, 1, 50, 0, 80, 69, 14, 51, 22, 29, 30, 0, 28, 0, 3, 0, 33, 69, 99, 8, 0, 0, 0, 45, 0, 90, 0, 3, 0, 0, 0, 0, 68, 99, 99, 38, 87",
  "Flugle Horn 2, 9, 0, 1, 80, 50, 0, 99, 0, 50, 0, 0, 50, 1, 0, 55, 21, 73, 23, 65, 10, 0, 53, 16, 3, 0, 29, 69, 72, 6, 99, 0, 28, 74, 0, 0, 0, 3, 0, 58, 0, 3, 17, 23, 30, 3, 91",
  "Flute 2, 10, 2, 1, 0, 0, 0, 99, 24, 50, 1, 0, 50, 1, 99, 48, 0, 64, 16, 0, 0, 8, 33, 18, 3, 0, 52, 69, 99, 5, 0, 0, 0, 89, 0, 0, 0, 3, 0, 58, 0, 3, 38, 57, 90, 6, 91",
  "Oboe, 11, 0, 2, 99, 0, 0, 99, 0, 50, 0, 0, 50, 1, 0, 68, 20, 50, 24, 0, 5, 0, 42, 0, 3, 0, 22, 96, 84, 0, 34, 0, 0, 99, 0, 26, 0, 3, 0, 0, 0, 0, 39, 56, 99, 16, 91",
  "Sax II, 12, 1, 3, 80, 0, 0, 99, 36, 50, 3, 1, 57, 0, 60, 43, 34, 67, 18, 25, 8, 4, 40, 9, 3, 0, 20, 63, 81, 1, 50, 0, 0, 24, 3, 21, 0, 3, 0, 0, 0, 0, 7, 99, 91, 12, 91",
  "String Ens. 2, 13, 1, 2, 18, 31, 0, 99, 12, 66, 1, 0, 50, 0, 51, 60, 0, 50, 23, 0, 0, 0, 49, 0, 3, 0, 47, 71, 71, 29, 0, 0, 0, 99, 0, 0, 0, 3, 0, 0, 0, 0, 0, 23, 11, 16, 0",
  "Cello, 14, 0, 2, 18, 6, 0, 99, 0, 49, 1, 0, 50, 0, 99, 58, 17, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 21, 0, 0, 93, 47, 32, 0, 45, 3, 6, 0, 0, 3, 0, 29, 11, 16, 0",
  "High Harmonic, 15, 1, 3, 99, 3, 0, 99, 12, 52, 1, 0, 50, 1, 29, 92, 0, 59, 15, 89, 0, 5, 42, 27, 3, 0, 54, 54, 54, 17, 0, 0, 37, 25, 0, 59, 52, 1, 0, 0, 0, 3, 0, 48, 50, 99, 0",
  "Bass II, 16, 0, 2, 0, 36, 1, 99, 8, 51, 2, 1, 56, 0, 99, 45, 0, 72, 0, 82, 0, 0, 36, 0, 3, 0, 0, 76, 0, 0, 0, 0, 73, 69, 0, 36, 0, 3, 0, 36, 0, 3, 0, 18, 0, 13, 84",
  "Synth Bass, 17, 0, 3, 0, 3, 1, 99, 0, 52, 0, 2, 50, 1, 99, 43, 32, 74, 49, 99, 0, 0, 52, 0, 3, 0, 16, 47, 0, 0, 0, 0, 46, 55, 0, 90, 0, 2, 2, 4, 0, 3, 9, 17, 0, 0, 0",
  "Elec Bass II, 18, 1, 2, 0, 81, 0, 67, 0, 99, 2, 0, 49, 1, 94, 26, 5, 69, 53, 55, 0, 0, 32, 0, 3, 0, 37, 55, 0, 7, 94, 0, 47, 99, 0, 0, 0, 1, 0, 0, 0, 0, 14, 99, 85, 30, 63",
  "Hammond I, 19, 1, 2, 34, 10, 0, 55, 36, 50, 1, 0, 50, 0, 99, 7, 0, 78, 65, 0, 15, 4, 11, 0, 3, 0, 0, 55, 85, 15, 0, 0, 42, 99, 3, 17, 0, 3, 0, 16, 0, 3, 0, 20, 0, 16, 99",
  "Hammond II, 20, 0, 3, 28, 5, 1, 59, 12, 50, 3, 1, 50, 1, 68, 23, 0, 84, 7, 0, 0, 5, 7, 0, 3, 0, 14, 81, 96, 56, 0, 0, 0, 32, 3, 26, 3, 3, 0, 99, 18, 1, 8, 95, 95, 19, 91",
  "Wurlitzer, 21, 2, 3, 46, 19, 1, 47, 0, 60, 1, 1, 50, 1, 53, 54, 22, 54, 76, 49, 0, 2, 99, 0, 3, 0, 0, 64, 15, 22, 40, 0, 22, 99, 0, 26, 0, 3, 0, 99, 3, 1, 3, 22, 9, 1, 91",
  "Church Org I, 22, 1, 2, 0, 0, 0, 99, 31, 75, 3, 0, 50, 0, 24, 18, 8, 80, 11, 0, 25, 8, 40, 9, 3, 0, 30, 87, 81, 3, 51, 0, 0, 97, 3, 21, 0, 2, 0, 0, 0, 0, 30, 34, 97, 19, 91",
  "Church Org II, 23, 1, 1, 94, 99, 0, 11, 36, 50, 2, 1, 50, 0, 62, 62, 7, 74, 49, 63, 81, 10, 41, 0, 3, 0, 5, 44, 90, 16, 38, 0, 47, 99, 14, 28, 14, 3, 0, 39, 0, 3, 26, 55, 73, 90, 0",
  "Synth, 24, 1, 3, 11, 25, 0, 99, 12, 75, 0, 0, 50, 0, 0, 37, 6, 51, 66, 40, 10, 19, 39, 18, 3, 0, 53, 69, 37, 28, 0, 0, 0, 99, 3, 26, 0, 3, 1, 60, 0, 3, 14, 82, 82, 57, 0",
  "Angel Voices, 25, 1, 3, 44, 5, 0, 99, 12, 68, 3, 0, 57, 0, 51, 59, 38, 51, 17, 0, 43, 76, 42, 0, 3, 0, 47, 71, 71, 24, 0, 0, 0, 44, 0, 0, 0, 3, 0, 0, 0, 0, 0, 23, 11, 16, 0",
  "Mini Moog, 26, 1, 2, 18, 31, 0, 99, 12, 50, 1, 0, 50, 0, 51, 55, 38, 99, 40, 0, 0, 0, 0, 0, 3, 0, 47, 71, 71, 29, 0, 0, 0, 66, 0, 67, 0, 3, 0, 0, 0, 0, 93, 23, 11, 16, 0",
  "Synth in 5ths 2, 27, 0, 1, 98, 99, 1, 99, 32, 48, 3, 1, 35, 1, 94, 22, 19, 73, 99, 55, 0, 0, 0, 0, 3, 0, 37, 8, 64, 8, 94, 0, 47, 52, 0, 0, 0, 1, 0, 0, 0, 1, 14, 99, 83, 30, 63",
  "Interlude, 28, 1, 2, 55, 4, 1, 99, 7, 67, 1, 0, 50, 1, 54, 51, 99, 99, 28, 1, 0, 0, 52, 0, 3, 0, 10, 85, 99, 7, 0, 0, 37, 34, 0, 90, 0, 2, 0, 0, 0, 3, 0, 96, 99, 99, 0",
  "Voices, 29, 1, 3, 55, 4, 0, 99, 12, 68, 3, 0, 50, 1, 99, 59, 4, 50, 31, 30, 23, 0, 34, 0, 3, 0, 46, 30, 99, 26, 0, 0, 37, 44, 0, 0, 0, 2, 0, 42, 0, 3, 0, 96, 99, 99, 0",
  "Martian Synth, 30, 2, 3, 81, 42, 1, 81, 12, 51, 3, 0, 50, 1, 0, 68, 33, 50, 30, 0, 99, 99, 99, 0, 0, 0, 0, 66, 69, 6, 81, 0, 81, 79, 99, 99, 0, 1, 20, 75, 0, 2, 0, 0, 81, 65, 81",
  "Syn Drum, 31, 1, 2, 0, 67, 1, 0, 7, 50, 2, 0, 50, 0, 0, 29, 99, 55, 55, 96, 0, 4, 33, 0, 3, 0, 0, 29, 0, 14, 99, 0, 99, 54, 30, 26, 17, 3, 0, 99, 18, 1, 6, 16, 0, 11, 99",
  "A Piano III, 32, 1, 3, 12, 4, 0, 85, 6, 70, 3, 1, 54, 0, 18, 75, 18, 49, 10, 23, 10, 0, 0, 0, 3, 0, 5, 47, 0, 12, 50, 0, 99, 48, 0, 0, 0, 3, 1, 20, 41, 3, 14, 72, 11, 0, 47",
  "Bells, 33, 0, 1, 0, 1, 0, 0, 24, 47, 2, 0, 50, 0, 86, 61, 6, 90, 57, 40, 99, 16, 0, 0, 3, 0, 0, 31, 0, 27, 71, 0, 57, 95, 0, 44, 0, 3, 0, 29, 0, 3, 0, 0, 6, 99, 48",
  "Clavinet II, 34, 1, 3, 0, 0, 0, 20, 12, 58, 2, 0, 50, 1, 99, 67, 0, 70, 15, 1, 23, 7, 99, 0, 3, 0, 0, 40, 43, 40, 0, 0, 37, 99, 0, 90, 0, 2, 0, 39, 0, 3, 0, 0, 0, 9, 0",
  "Toy Piano II, 35, 0, 1, 0, 0, 0, 96, 24, 47, 2, 0, 50, 0, 86, 66, 60, 72, 57, 96, 4, 16, 0, 0, 3, 0, 0, 32, 0, 28, 71, 0, 57, 34, 0, 44, 0, 3, 0, 29, 0, 3, 0, 0, 6, 99, 48",
  "Celeste, 36, 2, 3, 99, 0, 0, 99, 36, 50, 1, 1, 50, 0, 91, 71, 55, 41, 98, 60, 4, 0, 40, 0, 3, 0, 0, 21, 0, 24, 47, 0, 72, 47, 0, 42, 0, 3, 0, 28, 0, 3, 0, 0, 0, 31, 25",
  "Mirimba, 37, 0, 3, 47, 0, 0, 99, 12, 57, 2, 2, 50, 1, 57, 61, 11, 70, 15, 1, 23, 7, 52, 0, 3, 0, 0, 40, 9, 33, 0, 0, 37, 90, 0, 90, 0, 2, 0, 41, 0, 3, 0, 39, 0, 9, 0",
  "Orchestron II, 38, 1, 2, 53, 7, 0, 79, 36, 58, 2, 2, 50, 0, 53, 34, 33, 71, 56, 18, 0, 0, 99, 0, 3, 0, 0, 99, 15, 14, 40, 0, 22, 58, 0, 26, 0, 3, 0, 30, 3, 1, 24, 37, 9, 7, 91",
  "Horns IV, 39, 1, 2, 20, 99, 1, 99, 19, 45, 2, 2, 50, 1, 99, 35, 20, 72, 21, 0, 0, 0, 0, 0, 2, 0, 26, 31, 90, 0, 0, 1, 0, 28, 0, 54, 99, 3, 0, 31, 0, 0, 31, 99, 99, 0, 0",
  "Horn Chorus, 40, 2, 3, 94, 99, 0, 12, 36, 50, 2, 0, 50, 0, 99, 62, 7, 74, 49, 63, 81, 10, 41, 0, 3, 0, 45, 44, 90, 35, 0, 0, 47, 99, 14, 28, 14, 3, 0, 0, 0, 0, 26, 55, 73, 90, 0",
  "Baroque Trpet, 41, 1, 2, 78, 0, 0, 99, 31, 54, 2, 2, 50, 0, 99, 52, 32, 58, 28, 29, 12, 0, 28, 0, 3, 0, 47, 23, 45, 5, 0, 0, 0, 87, 0, 0, 0, 3, 0, 20, 0, 3, 1, 83, 99, 34, 87",
  "Bamboo Flute, 42, 2, 2, 31, 4, 0, 99, 33, 45, 2, 2, 48, 1, 45, 0, 10, 82, 32, 99, 34, 8, 40, 9, 3, 0, 34, 74, 99, 8, 50, 1, 0, 99, 3, 21, 0, 2, 1, 33, 0, 2, 34, 74, 99, 8, 50",
  "Pan Pipes, 43, 0, 0, 99, 0, 0, 0, 23, 27, 2, 0, 57, 0, 99, 38, 0, 59, 65, 11, 64, 7, 59, 11, 3, 0, 20, 69, 99, 4, 22, 0, 5, 99, 5, 60, 5, 3, 0, 58, 0, 3, 31, 57, 90, 12, 0",
  "Bright Oboe, 44, 1, 3, 99, 0, 0, 99, 0, 9, 0, 0, 50, 0, 0, 24, 26, 79, 48, 27, 99, 8, 40, 9, 3, 0, 21, 3, 96, 8, 34, 0, 0, 37, 3, 25, 0, 3, 0, 0, 0, 0, 0, 0, 98, 57, 91",
  "Country Fiddle, 45, 0, 2, 55, 4, 0, 90, 0, 67, 1, 1, 50, 1, 58, 61, 13, 54, 15, 1, 0, 0, 52, 0, 3, 0, 60, 30, 99, 28, 0, 0, 37, 81, 0, 90, 0, 2, 0, 0, 0, 3, 0, 96, 99, 99, 0",
  "Strings III, 46, 1, 2, 36, 11, 1, 99, 0, 38, 1, 0, 50, 0, 99, 49, 11, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 24, 0, 0, 51, 59, 32, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Bright Strings, 47, 0, 2, 99, 0, 1, 57, 5, 71, 1, 1, 78, 0, 99, 21, 24, 79, 23, 25, 0, 0, 52, 2, 3, 0, 0, 34, 14, 8, 0, 0, 91, 91, 0, 90, 0, 2, 0, 0, 1, 0, 0, 68, 0, 59, 0",
  "Bass III, 48, 0, 2, 69, 0, 1, 45, 0, 50, 1, 0, 50, 1, 71, 42, 15, 75, 43, 69, 0, 0, 54, 0, 3, 0, 0, 69, 17, 5, 99, 0, 28, 99, 0, 0, 0, 3, 1, 58, 0, 3, 0, 5, 8, 2, 91",
  "Synth Bass II, 49, 0, 1, 0, 0, 1, 81, 0, 50, 0, 0, 50, 1, 0, 15, 45, 86, 25, 81, 0, 38, 99, 81, 0, 0, 0, 99, 0, 0, 0, 0, 23, 72, 22, 4, 0, 3, 0, 81, 0, 1, 0, 56, 0, 81, 0",
  "Bowed Bass, 50, 0, 1, 99, 0, 0, 95, 0, 45, 1, 0, 50, 1, 64, 46, 38, 75, 42, 50, 0, 0, 52, 0, 3, 0, 0, 26, 59, 22, 0, 0, 38, 99, 0, 4, 0, 2, 0, 0, 0, 0, 0, 55, 21, 0, 0",
  "S D Hammond, 51, 1, 2, 28, 5, 1, 56, 24, 50, 3, 0, 50, 1, 60, 2, 9, 96, 11, 0, 0, 0, 7, 0, 3, 0, 14, 81, 96, 26, 0, 0, 0, 39, 0, 26, 3, 3, 0, 99, 18, 1, 7, 95, 95, 80, 91",
  "60s Organ II, 52, 1, 2, 53, 0, 0, 79, 36, 58, 2, 0, 50, 0, 53, 48, 21, 54, 76, 49, 0, 0, 99, 0, 3, 0, 23, 99, 15, 17, 40, 0, 22, 82, 0, 26, 0, 3, 0, 99, 3, 1, 3, 22, 9, 1, 91",
  "Church Org IV, 53, 1, 3, 28, 5, 1, 95, 24, 50, 3, 1, 50, 1, 99, 59, 0, 84, 7, 0, 0, 5, 7, 0, 3, 0, 14, 81, 96, 21, 0, 0, 0, 20, 0, 26, 3, 3, 0, 99, 18, 1, 8, 95, 95, 19, 91",
  "Harmonium, 54, 1, 3, 28, 50, 1, 56, 24, 50, 3, 1, 50, 1, 60, 2, 26, 99, 7, 25, 0, 0, 7, 0, 3, 0, 14, 81, 96, 36, 0, 0, 0, 28, 0, 26, 3, 3, 52, 99, 18, 1, 7, 95, 95, 46, 91",
  "Mod Synth, 55, 0, 2, 0, 0, 0, 99, 19, 57, 2, 2, 49, 0, 99, 70, 34, 40, 52, 57, 0, 0, 0, 0, 1, 0, 0, 99, 99, 0, 0, 0, 71, 59, 1, 37, 0, 3, 2, 39, 0, 3, 0, 46, 0, 19, 3",
  "Synth III, 56, 1, 2, 18, 31, 0, 99, 12, 66, 1, 0, 50, 0, 51, 30, 38, 99, 33, 0, 0, 0, 0, 0, 3, 0, 47, 71, 71, 20, 0, 0, 51, 41, 0, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Synth IV, 57, 1, 2, 36, 11, 1, 99, 0, 38, 1, 0, 50, 0, 99, 50, 51, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 20, 0, 0, 51, 41, 32, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Chimes, 58, 1, 0, 0, 0, 1, 62, 30, 50, 2, 2, 50, 1, 99, 59, 0, 45, 63, 0, 0, 0, 33, 0, 1, 0, 0, 47, 0, 51, 0, 0, 0, 99, 0, 26, 0, 3, 1, 16, 5, 1, 13, 86, 55, 71, 99",
  "Video Games, 59, 2, 3, 54, 99, 0, 99, 25, 50, 1, 2, 33, 0, 94, 76, 22, 0, 41, 55, 4, 0, 0, 0, 3, 0, 13, 24, 97, 23, 0, 0, 47, 52, 0, 0, 0, 1, 0, 0, 0, 1, 12, 14, 6, 4, 14",
  "Cat Fight, 60, 2, 2, 59, 99, 0, 99, 8, 65, 2, 0, 50, 0, 66, 61, 53, 73, 99, 0, 52, 26, 99, 0, 2, 0, 47, 71, 71, 20, 0, 0, 93, 99, 0, 99, 46, 2, 0, 99, 0, 3, 25, 90, 75, 84, 0",
  "Blossynth, 61, 2, 2, 18, 29, 0, 99, 24, 66, 1, 0, 50, 0, 51, 62, 25, 50, 35, 0, 0, 0, 45, 0, 3, 0, 47, 71, 71, 32, 0, 0, 0, 73, 0, 0, 0, 3, 0, 70, 0, 3, 0, 23, 11, 16, 0",
  "Reiteration, 62, 2, 2, 99, 99, 1, 99, 0, 50, 3, 1, 99, 1, 99, 68, 17, 75, 68, 99, 0, 14, 50, 0, 3, 0, 0, 76, 0, 22, 90, 0, 61, 46, 99, 50, 0, 2, 99, 50, 0, 3, 99, 72, 0, 79, 0",
  "Rise, 63, 0, 1, 0, 0, 1, 99, 0, 50, 1, 0, 50, 1, 99, 86, 74, 25, 71, 0, 10, 4, 99, 99, 0, 0, 0, 99, 0, 0, 0, 0, 23, 28, 8, 1, 0, 3, 0, 81, 0, 3, 0, 49, 0, 81, 0",
};

const String IRAM[64] = {
  "A Piano 16, 0, 1, 1, 99, 4, 1, 99, 35, 51, 1, 0, 49, 1, 32, 48, 8, 53, 90, 99, 26, 0, 0, 0, 3, 0, 10, 31, 0, 9, 90, 0, 99, 98, 0, 17, 0, 2, 0, 0, 0, 3, 14, 23, 11, 99, 0",
  "Piano 8, 1, 1, 3, 99, 0, 1, 99, 24, 60, 2, 0, 50, 1, 57, 28, 32, 69, 34, 0, 33, 7, 52, 0, 3, 0, 0, 11, 0, 9, 18, 0, 37, 99, 0, 90, 0, 2, 0, 0, 0, 3, 0, 19, 65, 49, 0",
  "Clavinet, 2, 1, 3, 78, 0, 0, 82, 19, 55, 2, 2, 50, 0, 99, 99, 22, 38, 43, 0, 47, 0, 0, 0, 3, 0, 17, 7, 0, 8, 99, 0, 37, 99, 0, 43, 0, 1, 0, 81, 0, 1, 10, 19, 99, 99, 0",
  "Harpsichord 2, 3, 1, 3, 99, 0, 0, 82, 17, 44, 3, 1, 40, 0, 99, 99, 41, 42, 99, 0, 0, 0, 0, 0, 3, 0, 0, 26, 0, 3, 53, 0, 54, 26, 0, 43, 0, 1, 0, 81, 0, 1, 0, 97, 0, 0, 87",
  "Jazz Guitar, 4, 2, 2, 21, 24, 0, 29, 36, 50, 2, 0, 50, 1, 99, 51, 0, 68, 21, 14, 23, 0, 51, 0, 3, 0, 0, 55, 99, 35, 0, 0, 37, 62, 0, 90, 0, 2, 0, 57, 0, 3, 0, 35, 9, 74, 0",
  "Rhodes, 5, 1, 2, 47, 0, 0, 99, 12, 57, 2, 0, 50, 1, 57, 64, 6, 70, 3, 38, 23, 7, 52, 0, 3, 0, 0, 33, 0, 15, 19, 0, 88, 79, 0, 90, 0, 2, 0, 0, 0, 3, 0, 24, 11, 9, 0",
  "French Horn, 6, 1, 1, 0, 50, 0, 99, 12, 60, 1, 0, 50, 1, 54, 43, 12, 77, 13, 36, 11, 0, 39, 0, 3, 0, 37, 69, 72, 10, 99, 0, 72, 59, 0, 0, 0, 3, 0, 0, 0, 3, 41, 47, 53, 9, 91",
  "Synth Horn, 7, 0, 3, 2, 0, 0, 99, 24, 50, 1, 1, 50, 0, 45, 35, 22, 74, 26, 18, 13, 8, 40, 10, 3, 0, 25, 63, 81, 1, 50, 0, 0, 41, 3, 21, 0, 3, 0, 0, 0, 0, 2, 51, 99, 8, 91",
  "Soundtrack, 8, 1, 2, 75, 12, 0, 99, 31, 50, 1, 1, 50, 0, 80, 69, 14, 51, 22, 29, 30, 0, 28, 0, 3, 0, 33, 69, 99, 8, 0, 0, 0, 45, 0, 90, 0, 3, 0, 0, 0, 0, 68, 99, 99, 38, 87",
  "Flugle Horn 2, 9, 0, 1, 80, 50, 0, 99, 0, 50, 0, 0, 50, 1, 0, 55, 21, 73, 23, 65, 10, 0, 53, 16, 3, 0, 29, 69, 72, 4, 99, 0, 28, 74, 0, 0, 0, 3, 0, 58, 0, 3, 17, 23, 30, 3, 91",
  "Flute 2, 10, 2, 1, 0, 0, 0, 99, 24, 50, 1, 0, 50, 1, 99, 48, 0, 64, 16, 0, 0, 8, 33, 18, 3, 0, 52, 69, 99, 4, 0, 0, 0, 89, 0, 0, 0, 3, 0, 58, 0, 3, 38, 57, 90, 6, 91",
  "Oboe, 11, 0, 2, 99, 0, 0, 99, 0, 50, 0, 0, 50, 1, 0, 68, 34, 50, 24, 0, 5, 0, 42, 0, 3, 0, 22, 96, 84, 0, 34, 0, 0, 99, 0, 26, 0, 3, 0, 0, 0, 0, 39, 56, 99, 16, 91",
  "Sax II, 12, 1, 3, 80, 0, 0, 99, 36, 50, 3, 1, 57, 0, 60, 43, 34, 67, 18, 25, 8, 4, 40, 9, 3, 0, 20, 63, 81, 1, 50, 0, 0, 18, 3, 21, 0, 3, 0, 0, 0, 0, 7, 99, 91, 12, 91",
  "String Ens. 2, 13, 1, 2, 18, 31, 0, 99, 12, 66, 1, 0, 50, 0, 51, 60, 0, 50, 23, 0, 0, 0, 49, 0, 3, 0, 47, 71, 71, 24, 0, 0, 0, 99, 0, 0, 0, 3, 0, 0, 0, 0, 0, 23, 11, 16, 0",
  "Cello, 14, 0, 2, 18, 6, 0, 99, 0, 49, 1, 0, 50, 0, 99, 58, 17, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 17, 0, 0, 93, 47, 32, 0, 45, 3, 6, 0, 0, 3, 0, 29, 11, 16, 0",
  "High Harmonic, 15, 1, 3, 99, 3, 0, 99, 12, 52, 1, 0, 50, 1, 29, 92, 0, 59, 15, 89, 0, 5, 42, 27, 3, 0, 54, 54, 54, 15, 0, 0, 37, 25, 0, 59, 52, 1, 0, 0, 0, 3, 0, 48, 50, 99, 0",
  "Bass II, 16, 0, 2, 0, 36, 1, 99, 8, 51, 2, 1, 56, 0, 99, 45, 0, 72, 0, 82, 0, 0, 36, 0, 3, 0, 0, 76, 0, 0, 0, 0, 73, 57, 0, 36, 0, 3, 0, 36, 0, 3, 0, 18, 0, 13, 84",
  "Synth Bass, 17, 0, 3, 0, 3, 1, 99, 0, 52, 0, 2, 50, 1, 99, 43, 32, 74, 49, 99, 0, 0, 52, 0, 3, 0, 16, 47, 0, 0, 0, 0, 46, 49, 0, 90, 0, 2, 2, 4, 0, 3, 9, 17, 0, 0, 0",
  "Elec Bass II, 18, 1, 2, 0, 81, 0, 67, 0, 99, 2, 0, 49, 1, 94, 26, 5, 69, 53, 55, 0, 0, 32, 0, 3, 0, 37, 55, 0, 5, 94, 0, 47, 99, 0, 0, 0, 1, 0, 0, 0, 0, 14, 99, 85, 30, 63",
  "Hammond I, 19, 1, 2, 34, 10, 0, 55, 36, 50, 1, 0, 50, 0, 99, 7, 0, 78, 65, 0, 15, 4, 11, 0, 3, 0, 0, 55, 85, 15, 0, 0, 42, 99, 3, 17, 0, 3, 0, 16, 0, 3, 0, 20, 0, 16, 99",
  "Hammond II, 20, 0, 3, 28, 5, 1, 59, 12, 50, 3, 1, 50, 1, 68, 23, 0, 84, 7, 0, 0, 5, 7, 0, 3, 0, 14, 81, 96, 55, 0, 0, 0, 30, 3, 26, 3, 3, 0, 99, 18, 1, 8, 95, 95, 19, 91",
  "Wurlitzer, 21, 2, 3, 46, 19, 1, 47, 0, 60, 1, 1, 50, 1, 53, 54, 22, 54, 76, 49, 0, 2, 99, 0, 3, 0, 0, 64, 15, 18, 40, 0, 22, 99, 0, 26, 0, 3, 0, 99, 3, 1, 3, 22, 9, 1, 91",
  "Church Org I, 22, 1, 2, 0, 0, 0, 99, 31, 75, 3, 0, 50, 0, 24, 18, 8, 80, 11, 0, 25, 8, 40, 9, 3, 0, 30, 87, 81, 1, 51, 0, 0, 97, 3, 21, 0, 2, 0, 0, 0, 0, 30, 34, 97, 19, 91",
  "Church Org II, 23, 1, 1, 94, 99, 0, 11, 36, 50, 2, 1, 50, 0, 62, 62, 7, 74, 49, 63, 81, 10, 41, 0, 3, 0, 5, 44, 90, 13, 38, 0, 47, 99, 14, 28, 14, 3, 0, 39, 0, 3, 26, 55, 73, 90, 0",
  "Synth, 24, 1, 3, 11, 25, 0, 99, 12, 75, 0, 0, 50, 0, 0, 37, 6, 51, 66, 40, 10, 19, 39, 18, 3, 0, 53, 69, 37, 25, 0, 0, 0, 99, 3, 26, 0, 3, 1, 60, 0, 3, 14, 82, 82, 57, 0",
  "Angel Voices, 25, 1, 3, 44, 5, 0, 99, 12, 68, 3, 0, 57, 0, 51, 59, 38, 51, 17, 0, 43, 76, 42, 0, 3, 0, 47, 71, 71, 24, 0, 0, 0, 44, 0, 0, 0, 3, 0, 0, 0, 0, 0, 23, 11, 16, 0",
  "Mini Moog, 26, 1, 2, 18, 31, 0, 99, 12, 50, 1, 0, 50, 0, 51, 55, 38, 99, 40, 0, 0, 0, 0, 0, 3, 0, 47, 71, 71, 24, 0, 0, 0, 66, 0, 67, 0, 3, 0, 0, 0, 0, 93, 23, 11, 16, 0",
  "Synth in 5ths 2, 27, 0, 1, 98, 99, 1, 99, 32, 48, 3, 1, 35, 1, 94, 22, 19, 73, 99, 55, 0, 0, 0, 0, 3, 0, 37, 8, 64, 5, 94, 0, 47, 52, 0, 0, 0, 1, 0, 0, 0, 1, 14, 99, 83, 30, 63",
  "Interlude, 28, 1, 2, 55, 4, 1, 99, 7, 67, 1, 0, 50, 1, 54, 51, 99, 99, 28, 1, 0, 0, 52, 0, 3, 0, 10, 85, 99, 5, 0, 0, 37, 33, 0, 90, 0, 2, 0, 0, 0, 3, 0, 96, 99, 99, 0",
  "Voices, 29, 1, 3, 55, 4, 0, 99, 12, 68, 3, 0, 50, 1, 99, 59, 4, 50, 31, 30, 23, 0, 34, 0, 3, 0, 46, 30, 99, 24, 0, 0, 37, 42, 0, 0, 0, 2, 0, 42, 0, 3, 0, 96, 99, 99, 0",
  "Martian Synth, 30, 2, 3, 81, 42, 1, 81, 12, 51, 3, 0, 50, 1, 0, 68, 33, 50, 30, 0, 99, 99, 99, 0, 0, 0, 0, 66, 69, 4, 81, 0, 81, 79, 99, 99, 0, 1, 20, 75, 0, 2, 0, 0, 81, 65, 81",
  "Syn Drum, 31, 1, 2, 0, 67, 1, 0, 7, 50, 2, 0, 50, 0, 0, 29, 99, 55, 55, 96, 0, 4, 33, 0, 3, 0, 0, 26, 0, 10, 99, 0, 99, 53, 30, 26, 17, 3, 0, 99, 18, 1, 6, 16, 0, 11, 99",
  "A Piano III, 32, 1, 3, 12, 4, 0, 85, 6, 70, 3, 1, 54, 0, 18, 75, 18, 49, 10, 23, 10, 0, 0, 0, 3, 0, 5, 47, 0, 9, 50, 0, 99, 48, 0, 0, 0, 3, 1, 20, 41, 3, 14, 72, 11, 0, 47",
  "Bells, 33, 0, 1, 0, 1, 0, 0, 24, 47, 2, 0, 50, 0, 86, 61, 6, 90, 57, 40, 99, 16, 0, 0, 3, 0, 0, 29, 0, 24, 71, 0, 57, 95, 0, 44, 0, 3, 0, 29, 0, 3, 0, 0, 6, 99, 48",
  "Clavinet II, 34, 1, 3, 0, 0, 0, 20, 12, 58, 2, 0, 50, 1, 99, 67, 0, 70, 15, 1, 23, 7, 99, 0, 3, 0, 0, 40, 43, 36, 0, 0, 37, 99, 0, 90, 0, 2, 0, 39, 0, 3, 0, 0, 0, 9, 0",
  "Toy Piano II, 35, 0, 1, 0, 0, 0, 96, 24, 47, 2, 0, 50, 0, 86, 66, 60, 72, 57, 96, 4, 16, 0, 0, 3, 0, 0, 29, 0, 24, 71, 0, 57, 34, 0, 44, 0, 3, 0, 29, 0, 3, 0, 0, 6, 99, 48",
  "Celeste, 36, 2, 3, 99, 0, 0, 99, 36, 50, 1, 1, 50, 0, 91, 71, 55, 41, 98, 60, 4, 0, 40, 0, 3, 0, 0, 18, 0, 21, 47, 0, 72, 47, 0, 42, 0, 3, 0, 28, 0, 3, 0, 0, 0, 31, 25",
  "Mirimba, 37, 0, 3, 47, 0, 0, 99, 12, 57, 2, 2, 50, 1, 57, 61, 11, 70, 15, 1, 23, 7, 52, 0, 3, 0, 0, 40, 9, 30, 0, 0, 37, 90, 0, 90, 0, 2, 0, 41, 0, 3, 0, 39, 0, 9, 0",
  "Orchestron II, 38, 1, 2, 53, 7, 0, 79, 36, 58, 2, 2, 50, 0, 53, 34, 33, 71, 56, 18, 0, 0, 99, 0, 3, 0, 0, 99, 15, 14, 40, 0, 22, 58, 0, 26, 0, 3, 0, 30, 3, 1, 24, 37, 9, 7, 91",
  "Horns IV, 39, 1, 2, 20, 99, 1, 99, 19, 45, 2, 2, 50, 1, 99, 35, 20, 72, 21, 0, 0, 0, 0, 0, 2, 0, 26, 30, 90, 0, 0, 1, 0, 28, 0, 54, 99, 3, 0, 31, 0, 0, 31, 99, 99, 0, 0",
  "Horn Chorus, 40, 2, 3, 94, 99, 0, 12, 36, 50, 2, 0, 50, 0, 99, 62, 7, 74, 49, 63, 81, 10, 41, 0, 3, 0, 45, 44, 90, 33, 0, 0, 47, 99, 14, 28, 14, 3, 0, 0, 0, 0, 26, 55, 73, 90, 0",
  "Baroque Trpet, 41, 1, 2, 78, 0, 0, 99, 31, 54, 2, 2, 50, 0, 99, 52, 32, 58, 28, 29, 12, 0, 28, 0, 3, 0, 47, 23, 45, 5, 0, 0, 0, 87, 0, 0, 0, 3, 0, 20, 0, 3, 1, 83, 99, 34, 87",
  "Bamboo Flute, 42, 2, 2, 31, 4, 0, 99, 33, 45, 2, 2, 48, 1, 45, 0, 10, 82, 32, 99, 34, 8, 40, 9, 3, 0, 34, 74, 99, 8, 50, 1, 0, 99, 3, 21, 0, 2, 1, 33, 0, 2, 34, 74, 99, 8, 50",
  "Pan Pipes, 43, 0, 0, 99, 0, 0, 0, 23, 27, 2, 0, 57, 0, 99, 38, 0, 59, 65, 11, 64, 7, 59, 11, 3, 0, 20, 69, 99, 4, 22, 0, 5, 99, 5, 60, 5, 3, 0, 58, 0, 3, 31, 57, 90, 12, 0",
  "Bright Oboe, 44, 1, 3, 99, 0, 0, 99, 0, 9, 0, 0, 50, 0, 0, 24, 26, 79, 48, 27, 99, 8, 40, 9, 3, 0, 21, 3, 96, 6, 34, 0, 0, 37, 3, 25, 0, 3, 0, 0, 0, 0, 0, 0, 98, 57, 91",
  "Country Fiddle, 45, 0, 2, 55, 4, 0, 90, 0, 67, 1, 1, 50, 1, 58, 61, 13, 54, 15, 1, 0, 0, 52, 0, 3, 0, 60, 30, 99, 24, 0, 0, 37, 81, 0, 90, 0, 2, 0, 0, 0, 3, 0, 96, 99, 99, 0",
  "Strings III, 46, 1, 2, 36, 11, 1, 99, 0, 38, 1, 0, 50, 0, 99, 49, 11, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 17, 0, 0, 51, 59, 32, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Bright Strings, 47, 0, 2, 99, 0, 1, 57, 5, 71, 1, 1, 78, 0, 99, 21, 24, 79, 23, 25, 0, 0, 52, 2, 3, 0, 0, 34, 14, 6, 0, 0, 91, 92, 0, 90, 0, 2, 0, 0, 1, 0, 0, 68, 0, 59, 0",
  "Bass III, 48, 0, 2, 69, 0, 1, 45, 0, 50, 1, 0, 50, 1, 71, 42, 15, 75, 43, 69, 0, 0, 54, 0, 3, 0, 0, 69, 17, 3, 99, 0, 28, 99, 0, 0, 0, 3, 1, 58, 0, 3, 0, 5, 8, 2, 91",
  "Synth Bass II, 49, 0, 1, 0, 0, 1, 81, 0, 50, 0, 0, 50, 1, 0, 15, 45, 86, 25, 81, 0, 38, 99, 81, 0, 0, 0, 99, 0, 0, 0, 0, 23, 72, 22, 4, 0, 3, 0, 81, 0, 1, 0, 56, 0, 81, 0",
  "Bowed Bass, 50, 0, 1, 99, 0, 0, 95, 0, 45, 1, 0, 50, 1, 64, 46, 38, 75, 42, 50, 0, 0, 52, 0, 3, 0, 0, 26, 59, 17, 0, 0, 38, 99, 0, 4, 0, 2, 0, 0, 0, 0, 0, 55, 21, 0, 0",
  "S D Hammond, 51, 1, 2, 28, 5, 1, 56, 24, 50, 3, 0, 50, 1, 60, 2, 9, 96, 11, 0, 0, 0, 7, 0, 3, 0, 14, 81, 96, 22, 0, 0, 0, 34, 0, 26, 3, 3, 0, 99, 18, 1, 7, 95, 95, 80, 91",
  "60s Organ II, 52, 1, 2, 53, 0, 0, 79, 36, 58, 2, 0, 50, 0, 53, 48, 21, 54, 76, 49, 0, 0, 99, 0, 3, 0, 23, 99, 15, 14, 40, 0, 22, 82, 0, 26, 0, 3, 0, 99, 3, 1, 3, 22, 9, 1, 91",
  "Church Org IV, 53, 1, 3, 28, 5, 1, 95, 24, 50, 3, 1, 50, 1, 99, 59, 0, 84, 7, 0, 0, 5, 7, 0, 3, 0, 14, 81, 96, 19, 0, 0, 0, 20, 0, 26, 3, 3, 0, 99, 18, 1, 8, 95, 95, 19, 91",
  "Harmonium, 54, 1, 3, 28, 50, 1, 56, 24, 50, 3, 1, 50, 1, 60, 2, 26, 99, 7, 25, 0, 0, 7, 0, 3, 0, 14, 81, 96, 36, 0, 0, 0, 28, 0, 26, 3, 3, 52, 99, 18, 1, 7, 95, 95, 46, 91",
  "Mod Synth, 55, 0, 2, 0, 0, 0, 99, 19, 57, 2, 2, 49, 0, 99, 70, 34, 40, 52, 57, 0, 0, 0, 0, 1, 0, 0, 99, 99, 0, 0, 0, 71, 59, 1, 37, 0, 3, 2, 39, 0, 3, 0, 46, 0, 19, 3",
  "Synth III, 56, 1, 2, 18, 31, 0, 99, 12, 66, 1, 0, 50, 0, 51, 30, 38, 99, 33, 0, 0, 0, 0, 0, 3, 0, 47, 71, 71, 24, 0, 0, 51, 43, 0, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Synth IV, 57, 1, 2, 36, 11, 1, 99, 0, 38, 1, 0, 50, 0, 99, 50, 51, 50, 89, 0, 0, 0, 41, 45, 3, 0, 47, 71, 71, 17, 0, 0, 51, 41, 32, 0, 45, 3, 0, 18, 0, 3, 0, 23, 11, 16, 0",
  "Chimes, 58, 1, 0, 0, 0, 1, 62, 30, 50, 2, 2, 50, 1, 99, 59, 0, 45, 63, 0, 0, 0, 33, 0, 1, 0, 0, 47, 0, 48, 0, 0, 0, 99, 0, 26, 0, 3, 1, 16, 5, 1, 13, 86, 55, 71, 99",
  "Video Games, 59, 2, 3, 54, 99, 0, 99, 25, 50, 1, 2, 33, 0, 94, 76, 22, 0, 41, 55, 4, 0, 0, 0, 3, 0, 13, 24, 97, 23, 0, 0, 47, 52, 0, 0, 0, 1, 0, 0, 0, 1, 12, 14, 6, 4, 14",
  "Cat Fight, 60, 2, 2, 59, 99, 0, 99, 8, 65, 2, 0, 50, 0, 66, 61, 53, 73, 99, 0, 52, 26, 99, 0, 2, 0, 47, 71, 71, 17, 0, 0, 93, 99, 0, 99, 46, 2, 0, 99, 0, 3, 25, 90, 75, 84, 0",
  "Blossynth, 61, 2, 2, 18, 29, 0, 99, 24, 66, 1, 0, 50, 0, 51, 62, 25, 50, 35, 0, 0, 0, 45, 0, 3, 0, 47, 71, 71, 24, 0, 0, 0, 67, 0, 0, 0, 3, 0, 70, 0, 3, 0, 23, 11, 16, 0",
  "Reiteration, 62, 2, 2, 99, 99, 1, 99, 0, 50, 3, 1, 99, 1, 99, 68, 17, 75, 68, 99, 0, 14, 50, 0, 3, 0, 0, 76, 0, 19, 90, 0, 61, 46, 99, 50, 0, 2, 99, 50, 0, 3, 99, 72, 0, 79, 0",
  "Rise, 63, 0, 1, 0, 0, 1, 99, 0, 50, 1, 0, 50, 1, 99, 86, 74, 25, 71, 0, 10, 4, 99, 99, 0, 0, 0, 99, 0, 0, 0, 0, 23, 27, 8, 1, 0, 3, 0, 81, 0, 3, 0, 49, 0, 81, 0",
};
