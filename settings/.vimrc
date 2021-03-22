" Configuration file for Vi Improved, save as ~/.vimrc to use.
" Written on 2014-07-16 by Miko Bartnicki <mikobartnicki@gmail.com>.

set nocompatible              " be iMproved, required
filetype on " required


" IMPORTANT: grep will sometimes skip displaying the file name if you
" " search in a singe file. This will confuse Latex-Suite. Set your grep
" " program to always generate a file-name.
set grepprg=grep\ -nH\ $*

" OPTIONAL: Starting with Vim 7, the filetype of empty .tex files defaults to
" " 'plaintex' instead of 'tex', which results in vim-latex not being loaded.
" " The following changes the default filetype back to 'tex':
let g:tex_flavor='latex'
let g:Tex_DefaultTargetFormat = 'pdf'
" set the runtime path to include Vundle and initialize
 set rtp+=~/.vim/bundle/Vundle.vim
 call vundle#begin()
 " alternatively, pass a path where Vundle should install plugins
 "call vundle#begin('~/some/path/here')

 " let Vundle manage Vundle, required
 Plugin 'VundleVim/Vundle.vim'
 Plugin 'scrooloose/nerdtree'
 "Plugin 'vim-syntastic/syntastic'
 Plugin 'scrooloose/nerdcommenter'
 Plugin 'christoomey/vim-tmux-navigator'
 Plugin 'DoxygenToolkit.vim'
 Plugin 'sirver/ultisnips'
 "Plugin 'Valloric/YouCompleteMe'
 "Plugin 'taketwo/vim-ros'
 Plugin 'honza/vim-snippets'
 Plugin 'ervandew/supertab'
 Plugin 'vim-airline/vim-airline'
 Plugin 'vim-airline/vim-airline-themes'
 Plugin 'tpope/vim-fugitive'
 Plugin 'octol/vim-cpp-enhanced-highlight'
 Plugin 'flazz/vim-colorschemes'
 Plugin 'nightsense/carbonized'
 Plugin 'nvie/vim-flake8'
 Plugin 'daylerees/colour-schemes'
 Plugin 'JamshedVesuna/vim-markdown-preview'
 Plugin 'rhysd/vim-clang-format'
 "Plugin 'suan/vim-instant-markdown'
call vundle#end()            " required

"let g:instant_markdown_autostart = 0
"let g:instant_markdown_slow = 1

" for the markdown visualization 
let vim_markdown_preview_hotkey='<C-m>'

let g:clang_format#style_options = {
            \ "AccessModifierOffset" : -4,
            \ "AllowShortIfStatementsOnASingleLine" : "true",
            \ "AlwaysBreakTemplateDeclarations" : "true",
            \ "Standard" : "C++11",
            \ "BreakBeforeBraces" : "Stroustrup"}


" map to <Leader>cf in C++ code
autocmd FileType c,cpp,objc nnoremap <buffer><Leader>cf :<C-u>ClangFormat<CR>
autocmd FileType c,cpp,objc vnoremap <buffer><Leader>cf :ClangFormat<CR>

filetype plugin indent on    " required
" use Vim mode instead of pure Vi, it must be the first instruction
set nocompatible
let python_highlight_all = 1

" display settings
set encoding=utf-8 " encoding used for displaying file
set ruler " show the cursor position all the time
set showmatch " highlight matching braces
set showmode " show insert/replace/visual mode
"colorscheme Tomorrow-Night-Blue
"colorscheme carbonized-dark

" write settings
set confirm " confirm :q in case of unsaved changes
set fileencoding=utf-8 " encoding used when saving file
set nobackup " do not keep the backup~ file

" search settings
set hlsearch " highlight search results

" file type specific settings
filetype on " enable file type detection
filetype plugin on " load the plugins for specific file types
filetype indent on " automatically indent code

" syntax highlighting
syntax enable " enable syntax highlighting
syntax on
set nu
au BufRead,BufNewFile *.cpp set filetype=cpp

" colors 
set t_Co=256
"colorscheme neon

" backspace configuration
set backspace=indent,eol,start

" Split navigation 
nnoremap <C-j> <C-W><C-J>
nnoremap <C-k> <C-W><C-K>
nnoremap <C-l> <C-W><C-L>
nnoremap <C-h> <C-W><C-H>

" set tags in recursive file search 
" set tags=./tags;~/Desktop/driveworks
" set tags=./tags;~/novatel_gavlab
" set tags=./tags;~/electric
" set tags=./tags;~/sick_ldmrs
set tags=./tags;~/Autoware
let g:cpp_experimental_template_highlight = 1
let g:cpp_class_decl_highlight = 1

" set ultisnip config 
" Trigger configuration. Do not use <tab> if you use https://github.com/Valloric/YouCompleteMe.
"let g:UltiSnipsExpandTrigger="<enter>"
let g:UltiSnipsJumpForwardTrigger="<c-b>"
let g:UltiSnipsJumpBackwardTrigger="<c-z>"

" If you want :UltiSnipsEdit to split your window.
let g:UltiSnipsEditSplit="vertical"


" Doxygen setup
let g:DoxygenToolkit_briefTag_pre="@Synopsis  "
let g:DoxygenToolkit_paramTag_pre="@Param "
let g:DoxygenToolkit_authorName="Paritosh Kelkar"

" make YCM compatible with UltiSnips (using supertab)
let g:ycm_key_list_select_completion = ['<C-n>', '<Down>']
let g:ycm_key_list_previous_completion = ['<C-p>', '<Up>']
let g:SuperTabDefaultCompletionType = '<C-n>'
 
" better key bindings for UltiSnipsExpandTrigger
let g:UltiSnipsSnippetDirectories = ['~/.vim/bundle/vim-snippets/UltiSnips', 'UltiSnips']
let g:UltiSnipsExpandTrigger="<C-j>"
let g:UltiSnipsJumpForwardTrigger="<c-j>"
let g:UltiSnipsJumpBackwardTrigger="<c-k>"

" let g:ycm_global_ycm_extra_conf = "~/catkin_ws/.ycm_extra_conf.py"
" let g:ycm_global_ycm_extra_conf = "~/.vim/.ycm_extra_conf.py"
" let g:ycm_global_ycm_extra_conf = "~/catkin_ws/.ycm_extra_conf.py"
let g:ycm_autoclose_preview_window_after_completion = 1

" airline 
set laststatus=2
let g:airline_powerline_fonts = 1
set encoding=utf-8
" Enable the list of buffers
let g:airline#extensions#tabline#enabled = 1
"
" " Show just the filename
let g:airline#extensions#tabline#fnamemod = ':t'
let g:airline#extensions#tabline#buffer_nr_show = 1

" switching between tabs
nnoremap <C-PageUp> :tabprevious<CR>
nnoremap <C-PageDown> :tabnext<CR>

" enhanced cpp syntax highlighting 
let g:cpp_class_scope_highlight = 1
let g:cpp_member_variable_highlight = 1

" buffer navigation 
" Mappings to access buffers (don't use "\p" because a
" delay before pressing "p" would accidentally paste).
" \l       : list buffers
" \b \f \g : go back/forward/last-used
" \1 \2 \3 : go to buffer 1/2/3 etc
nnoremap <Leader>l :ls<CR>
nnoremap <Leader>b :bp<CR>
nnoremap <Leader>f :bn<CR>
nnoremap <Leader>g :e#<CR>
nnoremap <Leader>1 :1b<CR>
nnoremap <Leader>2 :2b<CR>
nnoremap <Leader>3 :3b<CR>
nnoremap <Leader>4 :4b<CR>
nnoremap <Leader>5 :5b<CR>
nnoremap <Leader>6 :6b<CR>
nnoremap <Leader>7 :7b<CR>
nnoremap <Leader>8 :8b<CR>
nnoremap <Leader>9 :9b<CR>
nnoremap <Leader>0 :10b<CR>
" It's useful to show the buffer number in the status line.
set laststatus=2 statusline=%02n:%<%f\ %h%m%r%=%-14.(%l,%c%V%)\ %P

" misc
set cursorline   
set incsearch           " search as characters are entered

" python's sake
"set tabstop=4       " number of visual spaces per TAB
"set softtabstop=4   " number of spaces in tab when editing
"set expandtab       " tabs are spaces

" substitution 
nnoremap <Leader>s :%s/\<<C-r><C-w>\>/

" folding params
set foldmethod=indent   
set foldnestmax=10
set nofoldenable
set foldlevel=2

" C-s for saving 
" If the current buffer has never been saved, it will have no name,
" call the file browser to save it, otherwise just save it.
command -nargs=0 -bar Update if &modified 
                           \|    if empty(bufname('%'))
                           \|        browse confirm write
                           \|    else
                           \|        confirm write
                           \|    endif
                           \|endif
nnoremap <silent> <C-S> :<C-u>Update<CR>

inoremap <c-s> <Esc>:Update<CR>

" global copy paste 
vmap <C-c> "+y
nmap <C-v> "+p

" keep wiindow; buffer delete 
"here is a more exotic version of my original Kwbd script
"delete the buffer; keep windows; create a scratch buffer if no buffers left
function s:Kwbd(kwbdStage)
  if(a:kwbdStage == 1)
    if(!buflisted(winbufnr(0)))
      bd!
      return
    endif
    let s:kwbdBufNum = bufnr("%")
    let s:kwbdWinNum = winnr()
    windo call s:Kwbd(2)
    execute s:kwbdWinNum . 'wincmd w'
    let s:buflistedLeft = 0
    let s:bufFinalJump = 0
    let l:nBufs = bufnr("$")
    let l:i = 1
    while(l:i <= l:nBufs)
      if(l:i != s:kwbdBufNum)
        if(buflisted(l:i))
          let s:buflistedLeft = s:buflistedLeft + 1
        else
          if(bufexists(l:i) && !strlen(bufname(l:i)) && !s:bufFinalJump)
            let s:bufFinalJump = l:i
          endif
        endif
      endif
      let l:i = l:i + 1
    endwhile
    if(!s:buflistedLeft)
      if(s:bufFinalJump)
        windo if(buflisted(winbufnr(0))) | execute "b! " . s:bufFinalJump | endif
      else
        enew
        let l:newBuf = bufnr("%")
        windo if(buflisted(winbufnr(0))) | execute "b! " . l:newBuf | endif
      endif
      execute s:kwbdWinNum . 'wincmd w'
    endif
    if(buflisted(s:kwbdBufNum) || s:kwbdBufNum == bufnr("%"))
      execute "bd! " . s:kwbdBufNum
    endif
    if(!s:buflistedLeft)
      set buflisted
      set bufhidden=delete
      set buftype=
      setlocal noswapfile
    endif
  else
    if(bufnr("%") == s:kwbdBufNum)
      let prevbufvar = bufnr("#")
      if(prevbufvar > 0 && buflisted(prevbufvar) && prevbufvar != s:kwbdBufNum)
        b #
      else
        bn
      endif
    endif
  endif
endfunction

command! Kwbd call s:Kwbd(1)
nnoremap <silent> <Plug>Kwbd :<C-u>Kwbd<CR>

" Create a mapping (e.g. in your .vimrc) like this:
nmap <C-W>! <Plug>Kwbd

" insert current filename 
inoremap \fn <C-R>=expand("%:t:r")<CR>

set expandtab
set shiftwidth=2
set softtabstop=2

" map for 0 -> ^
nmap 0 ^

" colorcolumn 
set colorcolumn=80
set textwidth=80

" ctags 
noremap <Leader>t :!ctags-proj.sh<CR>

