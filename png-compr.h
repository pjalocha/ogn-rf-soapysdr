#include <stdint.h>
#include <string.h>
#include <png.h>

class PNG
{ private:

  public:
   uint8_t *Data;                                 // compressed PNG storage
   size_t   Size;                                 // actual PNG size: can be more Allocated

  private:
   size_t Allocated;                             // allocated storage
   static const size_t AllocUnit = 8192;         // storage will be allocated in blocks

   size_t Reallocate(size_t NewSize)            // reallocate JPEG storage to a new (bigger) size
   { // printf("Reallocate(%lu)\n", NewSize);
     Data=(uint8_t *)realloc(Data, NewSize);
     if(Data==0) { Allocated=0; Size=0; return 0; } // return zero if not possible to reallocate
     return Allocated=NewSize; }

  public:
   PNG()
   { Data=0; Allocated=0; Size=0; }

  ~PNG()
   { if(Data) free(Data); }

   static void Write(png_structp Png, png_bytep Byte, png_size_t Len)
   { // printf("Write(%016lX, , %lu)\n", (uint64_t)Png, Len);
     PNG *Client = (PNG *)png_get_io_ptr(Png); Client->Write(Byte, Len); }

   void Write(uint8_t *Byte, int Len)
   { // printf("Write( , %d) Size=%lu\n", Len, Size);
     if(Size+Len>Allocated)
     { int Blocks=(Size+Len+AllocUnit-1)/AllocUnit; Reallocate(Blocks*AllocUnit); }
     memcpy(Data+Size, Byte, Len); Size+=Len; }

   // static void Flush(png_structp Png)
   // { PNG *Client = (PNG *)png_get_io_ptr(Png); Client->Flush(); }

   // void Flush(void)
   // { printf("Flush() Size=%lu\n", Size); }

   int Compress_MONO8(uint8_t *Image, int Width, int Height, int Rot=0)        // compress a grey-scale image
   { Size=0;

     png_structp Struct = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
     if(Struct==0) return 0;
     png_infop Info = png_create_info_struct(Struct);
     if(Info==0) { png_destroy_write_struct(&Struct, (png_infopp)0); return 0; }
     if(setjmp(png_jmpbuf(Struct))) { png_destroy_write_struct(&Struct, (png_infopp)0); return 0; }
     png_set_write_fn(Struct, this, Write, 0 /* Flush */ );

     if(Rot)
     { png_set_IHDR(Struct, Info, Height, Width,
                    8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                    PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
       png_write_info(Struct, Info);
       uint8_t Line[Height];
       for(int Row=0; Row<Width; Row++)
       { uint8_t *Src = Image + (Width-Row-1);
         uint8_t *Dst = Line;
         for(int Pixel=0; Pixel<Height; Pixel++)
         { *Dst++ = *Src; Src+=Width; }
         png_write_row(Struct, Line); }
     }
     else
     { png_set_IHDR(Struct, Info, Width, Height,
                    8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                    PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
       // printf("Size=%lu this=%016lX\n", Size, (uint64_t)this);
       png_write_info(Struct, Info);
       for(int Row=0; Row<Height; Row++)
       { // printf("Row %3d: Size=%lu\n", Row, Size);
         png_write_row(Struct, Image+(Row*Width)); }
     }

     png_write_end(Struct, Info);
     png_destroy_write_struct(&Struct, (png_infopp)0);
     // printf("Compress_MONO8( , %d, %d) Size=%lu\n", Width, Height, Size);
     return Size; }

   int Write(const char *FileName)                                        // write compressed JPEG image to a file
   { // printf("Write(%s) Size=%lu\n", FileName, Size);
     if(Data==0) return 0;
     FILE *File = fopen(FileName, "wb"); if(File==0) return 0;
     size_t Written=fwrite(Data, 1, Size, File);
     fclose(File); return Written; }

} ;

